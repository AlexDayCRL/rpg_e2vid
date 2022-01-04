from .path_utils import ensure_dir
import cv2
from os.path import join
import atexit
import numpy as np
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import rosbag

def make_event_preview(events, mode='red-blue', num_bins_to_show=-1):
    # events: [1 x C x H x W] event tensor
    # mode: 'red-blue' or 'grayscale'
    # num_bins_to_show: number of bins of the voxel grid to show. -1 means show all bins.
    assert(mode in ['red-blue', 'grayscale'])
    if num_bins_to_show < 0:
        sum_events = torch.sum(events[0, :, :, :], dim=0).detach().cpu().numpy()
    else:
        sum_events = torch.sum(events[0, -num_bins_to_show:, :, :], dim=0).detach().cpu().numpy()

    if mode == 'red-blue':
        # Red-blue mode
        # positive events: blue, negative events: red
        event_preview = np.zeros((sum_events.shape[0], sum_events.shape[1], 3), dtype=np.uint8)
        b = event_preview[:, :, 0]
        r = event_preview[:, :, 2]
        b[sum_events > 0] = 255
        r[sum_events < 0] = 255
    else:
        # Grayscale mode
        # normalize event image to [0, 255] for display
        m, M = -10.0, 10.0
        event_preview = np.clip((255.0 * (sum_events - m) / (M - m)).astype(np.uint8), 0, 255)

    return event_preview

class RosbagWriter:
    """
    Utility class to write images to a rosbag file
    """

    def __init__(self, options):
        self.output_rosbag = options.output_rosbag
        self.save_events = options.show_events
        self.event_display_mode = options.event_display_mode
        self.num_bins_to_show = options.num_bins_to_show
        print('== Rosbag Writer ==')
        if self.output_rosbag:
            print(f"Will write images back to: {self.output_rosbag}")
            try:
                self.bag = rosbag.Bag(self.output_rosbag, 'a')
            except rosbag.bag.ROSBagException:
                self.bag = rosbag.Bag(self.output_rosbag, 'w')
            self.bridge = CvBridge()

            if self.save_events:
                print(f'Will write event previews to: {self.output_rosbag}')

            atexit.register(self.__cleanup__)
        else:
            print('Will not write images to disk.')

    def __call__(self, img, event_tensor_id, stamp=None, events=None):
        if not self.output_rosbag:
            return

        if self.save_events and events is not None:
            event_preview = make_event_preview(events, mode=self.event_display_mode,
                                               num_bins_to_show=self.num_bins_to_show)
            msg = self.bridge.cv2_to_imgmsg(event_preview, "rgb8")
            msg.header.stamp = rospy.Time.from_sec(stamp / 1e9)

            self.bag.write("/crl_rzr/dvs/event_viz", msg, msg.header.stamp)

        msg = self.bridge.cv2_to_imgmsg(img, "mono8")
        msg.header.stamp = rospy.Time.from_sec(stamp / 1e9)
        self.bag.write("/crl_rzr/dvs/image", msg, msg.header.stamp)

    def __cleanup__(self):
        if self.output_rosbag:
            self.bag.close()

class ImagePublisher:
    """
    Utility class to publish images over ros
    """

    def __init__(self, options):

        self.topic = options.topic
        self.publish_events = options.show_events
        self.event_display_mode = options.event_display_mode
        self.num_bins_to_show = options.num_bins_to_show
        print('== Image Publisher ==')
        if self.topic:
            rospy.init_node("e2vid", anonymous=True)
            self.bridge = CvBridge()
            self.image_topic = self.topic + "/image"
            self.image_publisher = rospy.Publisher(self.image_topic, Image)
            print(f"Will publish images to: {self.image_topic}")

            if self.publish_events:
                self.event_topic = self.topic + "/event"
                self.event_publisher = rospy.Publisher(self.event_topic, Image)
                print(f'Will publish event previews to: {self.event_topic}')
        else:
            print('Will not publish images to ROS.')

    def __call__(self, img, event_tensor_id, stamp=None, events=None):
        if not self.topic:
            return

        if self.publish_events and events is not None:
            event_preview = make_event_preview(events, mode=self.event_display_mode,
                                               num_bins_to_show=self.num_bins_to_show)
            self.event_publisher.publish(self.bridge.cv2_to_imgmsg(event_preview, "rgb8"))


        self.image_publisher.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))


class ImageWriter:
    """
    Utility class to write images to disk.
    Also writes the image timestamps into a text file.
    """

    def __init__(self, options):

        self.output_folder = options.output_folder
        self.dataset_name = options.dataset_name
        self.save_events = options.show_events
        self.event_display_mode = options.event_display_mode
        self.num_bins_to_show = options.num_bins_to_show
        print('== Image Writer ==')
        if self.output_folder:
            ensure_dir(self.output_folder)
            ensure_dir(join(self.output_folder, self.dataset_name))
            print('Will write images to: {}'.format(join(self.output_folder, self.dataset_name)))
            self.timestamps_file = open(join(self.output_folder, self.dataset_name, 'timestamps.txt'), 'a')

            if self.save_events:
                self.event_previews_folder = join(self.output_folder, self.dataset_name, 'events')
                ensure_dir(self.event_previews_folder)
                print('Will write event previews to: {}'.format(self.event_previews_folder))

            atexit.register(self.__cleanup__)
        else:
            print('Will not write images to disk.')

    def __call__(self, img, event_tensor_id, stamp=None, events=None):
        if not self.output_folder:
            return

        if self.save_events and events is not None:
            event_preview = make_event_preview(events, mode=self.event_display_mode,
                                               num_bins_to_show=self.num_bins_to_show)
            cv2.imwrite(join(self.event_previews_folder,
                             'events_{:010d}.png'.format(event_tensor_id)), event_preview)

        cv2.imwrite(join(self.output_folder, self.dataset_name,
                         'frame_{:010d}.png'.format(event_tensor_id)), img)
        if stamp is not None:
            self.timestamps_file.write('{:.18f}\n'.format(stamp))

    def __cleanup__(self):
        if self.output_folder:
            self.timestamps_file.close()


class ImageDisplay:
    """
    Utility class to display image reconstructions
    """

    def __init__(self, options):
        self.display = options.display
        self.show_events = options.show_events
        self.color = options.color
        self.event_display_mode = options.event_display_mode
        self.num_bins_to_show = options.num_bins_to_show

        self.window_name = 'Reconstruction'
        if self.show_events:
            self.window_name = 'Events | ' + self.window_name

        if self.display:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        self.border = options.display_border_crop
        self.wait_time = options.display_wait_time

    def crop_outer_border(self, img, border):
        if self.border == 0:
            return img
        else:
            return img[border:-border, border:-border]

    def __call__(self, img, events=None):

        if not self.display:
            return

        img = self.crop_outer_border(img, self.border)

        if self.show_events:
            assert(events is not None)
            event_preview = make_event_preview(events, mode=self.event_display_mode,
                                               num_bins_to_show=self.num_bins_to_show)
            event_preview = self.crop_outer_border(event_preview, self.border)

        if self.show_events:
            img_is_color = (len(img.shape) == 3)
            preview_is_color = (len(event_preview.shape) == 3)

            if(preview_is_color and not img_is_color):
                img = np.dstack([img] * 3)
            elif(img_is_color and not preview_is_color):
                event_preview = np.dstack([event_preview] * 3)

            img = np.hstack([event_preview, img])

        cv2.imshow(self.window_name, img)
        cv2.waitKey(self.wait_time)
