# Offline Event Camera Rendering Tool

Based on code provided with the following paper: [High Speed and High Dynamic Range Video with an Event Camera](http://rpg.ifi.uzh.ch/docs/TPAMI19_Rebecq.pdf). This tool allows (primarily) for offline rendering of DVS events. The use case for this tool is calibration. A bag will be collected with the events for  the calibration and then these events will be rendered so that the images can be used in a calibration pipeline.

## Install

Dependencies:

- [PyTorch](https://pytorch.org/get-started/locally/) >= 1.0
- [NumPy](https://www.numpy.org/)
- [Pandas](https://pandas.pydata.org/)
- [OpenCV](https://opencv.org/)
- [ROS](https://ros.org)


## Run

- Download the pretrained model:

```bash
wget "http://rpg.ifi.uzh.ch/data/E2VID/models/E2VID_lightweight.pth.tar" -O pretrained/E2VID_lightweight.pth.tar
```

- Download a log with the event camera data

- Run reconstruction:

```bash
python run_reconstruction.py \
  -c pretrained/E2VID_lightweight.pth.tar \
  --input_file "in.bag"
  --event_reader "rosbag"
  --output_rosbag "out.bag"
```

```bash
python run_reconstruction.py \
  -c pretrained/E2VID_lightweight.pth.tar \
  --input_file "in.bag"
  --event_reader "rosbag"
  --output_folder "./out/"
```

## Parameters

Below is a description of the most important parameters:

#### Main parameters

- ``--window_size`` / ``-N`` (default: None) Number of events per window. This is the parameter that has the most influence of the image reconstruction quality. If set to None, this number will be automatically computed based on the sensor size, as N = width * height * num_events_per_pixel (see description of that parameter below). Ignored if `--fixed_duration` is set.
- ``--fixed_duration`` (default: False) If True, will use windows of events with a fixed duration (i.e. a fixed output frame rate).
- ``--window_duration`` / ``-T`` (default: 33 ms) Duration of each event window, in milliseconds. The value of this parameter has strong influence on the image reconstruction quality. Its value may need to be adapted to the dynamics of the scene. Ignored if `--fixed_duration` is not set.
- ``--Imin`` (default: 0.0), `--Imax` (default: 1.0): linear tone mapping is performed by normalizing the output image as follows: `I = (I - Imin) / (Imax - Imin)`. If `--auto_hdr` is set to True, `--Imin` and `--Imax` will be automatically computed as the min (resp. max) intensity values.
- ``--auto_hdr`` (default: False) Automatically compute `--Imin` and `--Imax`. Disabled when `--color` is set.
- ``--color`` (default: False): if True, will perform color reconstruction as described in the paper. Only use this with a [color event camera](http://rpg.ifi.uzh.ch/CED.html) such as the Color DAVIS346.
- ``--event_reader`` how to treat the input file
- ``--input_topic`` ROS topic to read in the events

#### Output parameters

- ``--output_folder``: path of the output folder. If not set, the image reconstructions will not be saved to disk.
- ``--output_rosbag``: path of the rosbag to output to. This rosbag will contain ONLY the rendered images
- ``--output_topic``: topic to publish "live" viz on. Do not expect good FPS
- ``--dataset_name``: name of the output folder directory (default: 'reconstruction').

#### Display parameters

- ``--display`` (default: False): display the video reconstruction in real-time in an OpenCV window.
- ``--show_events`` (default: False): show the input events side-by-side with the reconstruction. If ``--output_folder`` is set, the previews will also be saved to disk in ``/path/to/output/folder/events``.

#### Additional parameters

- ``--num_events_per_pixel`` (default: 0.35): Parameter used to automatically estimate the window size based on the sensor size. The value of 0.35 was chosen to correspond to ~ 15,000 events on a 240x180 sensor such as the DAVIS240C.
- ``--no-normalize`` (default: False): Disable event tensor normalization: this will improve speed a bit, but might degrade the image quality a bit.
- ``--no-recurrent`` (default: False): Disable the recurrent connection (i.e. do not maintain a state). For experimenting only, the results will be flickering a lot.
- ``--hot_pixels_file`` (default: None): Path to a file specifying the locations of hot pixels (such a file can be obtained with [this tool](https://github.com/cedric-scheerlinck/dvs_tools/tree/master/dvs_hot_pixel_filter) for example). These pixels will be ignored (i.e. zeroed out in the event tensors).

## Acknowledgements

This code borrows from the following open source projects, whom we would like to thank:

- [pytorch-template](https://github.com/victoresque/pytorch-template)
