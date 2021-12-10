import pandas as pd
import zipfile
from os.path import splitext
import numpy as np
from .timers import Timer

import rosbag
import rospy
from rospy.msg import AnyMsg
# from dvs_msgs.msg import EventArray, Event


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

    def __init__(self, bag_path, topic, num_events=10000):
        self.bag = rosbag.Bag(bag_path)
        self.messages = self.bag.read_messages(topics=[topic], raw=True)
        self.event_array_msg = ROSEventArray(num_events)
        self.num_events = num_events

    def read_event_array(self, data):
        self.event_array_msg.set_dtype(data)
        event_array = np.frombuffer(data, dtype=self.event_array_msg.dtype(), count=1)
        events = event_array['events'].squeeze()
        events = np.array(events.tolist())

        event_buffer = np.stack([events[:, 2]*1e9 + events[:, 3], events[:, 0], events[:, 1], events[:, 4]]).T

        return event_buffer

    def __iter__(self):
        return self

    def __next__(self):
        with Timer('Reading events from event_buffer'):
            bag_msg = next(self.messages)
            event_window = self.read_event_array(bag_msg.message[1])
        return event_window


class RosSubscriberEventReader:
    """
    Reads events from live or playback data.
    """

    def __init__(self, topic, num_events=10000):
        rospy.init_node('event_reader', anonymous=True)
        self.sub = rospy.Subscriber(topic, AnyMsg, self.callback, queue_size=1000)
        self.event_buffer = None
        self.event_array_msg = ROSEventArray(num_events)
        self.num_events = num_events

    def callback(self, msg):
        with Timer('Responding to callbacks'):
            self.event_array_msg.set_dtype(msg._buff)
            event_array = np.frombuffer(msg._buff, dtype=self.event_array_msg.dtype(), count=1)
            events = event_array['events'].squeeze()
            events = np.array(events.tolist())

            event_buffer = np.stack([events[:, 2]*1e9 + events[:, 3], events[:, 0], events[:, 1], events[:, 4]]).T

            if self.event_buffer is not None:
                self.event_buffer = np.concatenate((self.event_buffer, event_buffer))
            else:
                self.event_buffer = event_buffer

    def __iter__(self):
        return self

    def __next__(self):
        with Timer('Reading events from event_buffer'):
            event_window = self.event_buffer
            self.event_buffer = None
        return event_window


class FixedSizeEventReader:
    """
    Reads events from a '.txt' or '.zip' file, and packages the events into
    non-overlapping event windows, each containing a fixed number of events.
    """

    def __init__(self, path_to_event_file, num_events=10000, start_index=0):
        print('Will use fixed size event windows with {} events'.format(num_events))
        print('Output frame rate: variable')
        self.iterator = pd.read_csv(path_to_event_file, delim_whitespace=True, header=None,
                                    names=['t', 'x', 'y', 'pol'],
                                    dtype={'t': np.float64, 'x': np.int16, 'y': np.int16, 'pol': np.int16},
                                    engine='c',
                                    skiprows=start_index + 1, chunksize=num_events, nrows=None, memory_map=True)

    def __iter__(self):
        return self

    def __next__(self):
        with Timer('Reading event window from file'):
            event_window = self.iterator.__next__().values
        return event_window


class FixedDurationEventReader:
    """
    Reads events from a '.txt' or '.zip' file, and packages the events into
    non-overlapping event windows, each of a fixed duration.

    **Note**: This reader is much slower than the FixedSizeEventReader.
              The reason is that the latter can use Pandas' very efficient cunk-based reading scheme implemented in C.
    """

    def __init__(self, path_to_event_file, duration_ms=50.0, start_index=0):
        print('Will use fixed duration event windows of size {:.2f} ms'.format(duration_ms))
        print('Output frame rate: {:.1f} Hz'.format(1000.0 / duration_ms))
        file_extension = splitext(path_to_event_file)[1]
        assert(file_extension in ['.txt', '.zip'])
        self.is_zip_file = (file_extension == '.zip')

        if self.is_zip_file:  # '.zip'
            self.zip_file = zipfile.ZipFile(path_to_event_file)
            files_in_archive = self.zip_file.namelist()
            assert(len(files_in_archive) == 1)  # make sure there is only one text file in the archive
            self.event_file = self.zip_file.open(files_in_archive[0], 'r')
        else:
            self.event_file = open(path_to_event_file, 'r')

        # ignore header + the first start_index lines
        for i in range(1 + start_index):
            self.event_file.readline()

        self.last_stamp = None
        self.duration_s = duration_ms / 1000.0

    def __iter__(self):
        return self

    def __del__(self):
        if self.is_zip_file:
            self.zip_file.close()

        self.event_file.close()

    def __next__(self):
        with Timer('Reading event window from file'):
            event_list = []
            for line in self.event_file:
                if self.is_zip_file:
                    line = line.decode("utf-8")
                t, x, y, pol = line.split(' ')
                t, x, y, pol = float(t), int(x), int(y), int(pol)
                event_list.append([t, x, y, pol])
                if self.last_stamp is None:
                    self.last_stamp = t
                if t > self.last_stamp + self.duration_s:
                    self.last_stamp = t
                    event_window = np.array(event_list)
                    return event_window

        raise StopIteration
