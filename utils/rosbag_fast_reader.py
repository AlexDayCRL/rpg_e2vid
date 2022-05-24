# %%
# ROSBag -> Numpy taken from https://github.com/jackg0/rpg_e2vid/commit/5ddef3df0b1bc97f0ad079979b6f1dbb5e695aa5
import rosbag
from rospy.msg import AnyMsg
from dvs_msgs.msg import EventArray, Event
import numpy as np
from tqdm import tqdm

class ROSHeader:
    def __init__(self):
        self.np_dtype = None

    def set_dtype(self, frame_bytes):
        frame_id_length = np.frombuffer(frame_bytes, dtype='<u4', count=1, offset=12).item()
        names = ['seq', 'secs', 'nsecs', 'frame_id_length', 'frame_id']
        formats = ['<u4', '<u4', '<u4', '<u4', f'<S{frame_id_length}']

        self.np_dtype = np.dtype(list(zip(names, formats)))

    def dtype(self):
        return self.np_dtype

class ROSEvent:
    def __init__(self):
        self.np_dtype = None
        names = ['x', 'y', 's', 'ns', 'polarity']
        formats = ['<u2', '<u2,', '<u4', '<u4,', '<u1']
        self.np_dtype = np.dtype(list(zip(names, formats)))

    def dtype(self):
        return self.np_dtype


class ROSEventArray:
    def __init__(self, max_num_events):
        self.np_dtype = None
        self.event = ROSEvent()
        self.header = ROSHeader()
        self.max_num_events = max_num_events

    def set_dtype(self, frame_bytes):
        self.header.set_dtype(frame_bytes)
        num_events = np.frombuffer(frame_bytes,
                                   dtype='<u4',
                                   count=1,
                                   offset=self.header.dtype().itemsize + 8).item()

        max_num_events = min(self.max_num_events, num_events)

        names = ['header', 'height', 'width', 'num_events', 'events']
        formats = [self.header.dtype(), '<u4', '<u4', '<u4', (self.event.dtype(), (max_num_events,))]

        self.np_dtype = np.dtype(list(zip(names, formats)))

    def dtype(self):
        return self.np_dtype

class RosbagEventReader:
    """
    Reads events from live or playback data stored in a rosbag.
    """

    def __init__(self, bag_path: str, topic: str, num_events: int=10000, num_messages: int=None):
        self.bag = rosbag.Bag(bag_path)
        self.topic = topic
        self.messages = self.bag.read_messages(topics=[topic], raw=True)
        self.num_messages = self.bag.get_message_count(topic)
        self.event_array_msg = ROSEventArray(1e99)
        self.num_events = num_events
        self.num_messages = num_messages

        if num_events and num_messages:
            self.num_events = None

        self.leftovers = None

    def total(self) -> int:
        total = 0
        for _, msg, _ in tqdm(self.bag.read_messages(topics=self.topic),
                              total=len(list(self.messages)),
                              desc="Enumerating messages to calculate total events"):
            total += len(msg.events)

        return total

    def read_event_array(self, data) -> np.array:
        self.event_array_msg.set_dtype(data)
        event_array = np.frombuffer(data, dtype=self.event_array_msg.dtype(), count=1)
        events = event_array['events'].squeeze()
        events = np.array(events.tolist())

        event_buffer = np.stack([events[:, 2]*1e9 + events[:, 3], events[:, 0], events[:, 1], events[:, 4]]).T

        return event_buffer

    def __iter__(self):
        return self

    def __next__(self) -> np.array:
        if self.num_messages:
            return np.vstack([self.read_event_array(next(self.messages).message[1]) for _ in range(self.num_messages)])

        buffer = None
        # Check if there are any leftovers
        if self.leftovers is not None and len(self.leftovers) >= 0:
            # Add no more than num_events to the buffer and remove them from the leftovers
            idx = min(self.num_events, len(self.leftovers))
            buffer = self.leftovers[:idx]
            self.leftovers = self.leftovers[idx:]

        # If the buffer doesn't contain num_events then:
        # 1. self.leftovers is empty
        # 2. we need to read the rest of the events from the next message 
        # 3. we need to put the leftover events in the leftovers array
        while buffer is None or len(buffer) != self.num_events:
            bag_msg = next(self.messages)
            event_window = self.read_event_array(bag_msg.message[1])

            if buffer is None:
                needed = self.num_events
                buffer = event_window[:needed]
            else:
                needed = self.num_events - len(buffer)
                buffer = np.vstack((buffer, event_window[:needed]))
        
            if self.leftovers is None:
                self.leftovers = event_window[needed:]
            else:
                self.leftovers = np.vstack((self.leftovers, event_window[needed:]))

        return buffer
