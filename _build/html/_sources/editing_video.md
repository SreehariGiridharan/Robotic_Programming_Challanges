# Editing and embedding a video

The following tutorial explains how you can

- edit your recorded video
- convert it to an appropriate format
- embed the output in your text.

## Editing

We will use ffmpeg on the shell to edit and convert the video to a format playable by the browser. ffmpeg project includes  with three tools:

1. `ffprobe` for getting info about the media
1. `ffplay` for trying the filters for editing
1. `ffmpeg` for applying the filters and converting

(rotation-of-videos-on-smartphones)=
### Rotation of videos on smartphones

You will very likely use your smartphone for recording. You may shot your video in portrait or landscape mode, which may cause some problems while editing your video with `ffmpeg`. So you should know how rotation works.

Even you record your video in landscape or portrait mode the smartphone camera will not notice the orientation and record what it sees through the lens and saves the results into memory. For example assume that your camera sees the world in the right orientation if you hold your smartphone upright. If you then rotate your phone 90 degrees counterclockwise then the objects which are on the left of your sight will be stored first and the resulting image in the memory will appear rotated 90 degrees counterclockwise. To correct this the help of the orientation sensor in your smartphone is attached to your image as meta data. Here is an example of such metadata:

```bash
ffprobe -hide_banner IN.mp4
```
```
...
Side data:
      displaymatrix: rotation of -90.00 degrees
...
```

The metadata indicates that the media should be presented using 90 degrees clockwise rotation.

However there are viewers which do not support this metadata which may lead to rotation errors.

#### Why rotation using metadata?

Probably the camera has dedicated encoder chip which can record and encode the frames to a compressed video format. The encoding hardware is hardwired so the same pixels on the camera photo sensor chip are stored at the same memory location of the encoder hardware which makes rotation not possible. It is much easier to do the rotation while viewing the image.

### Editing and converting to WebM container format with `ffmpeg`

Using `ffmpeg` tool you can edit and convert your video to a format supported by web browsers.

#### Editing
`ffmpeg` supports [various filters](https://ffmpeg.org/ffmpeg-filters.html). Some of them which are useful are:

- [crop](https://ffmpeg.org/ffmpeg-filters.html#crop) for cutting away unwanted parts of the video frames
- [scale](https://ffmpeg.org/ffmpeg-filters.html#scale) for scaling down the resolution to decrease the size of the output video
- [rotate](https://ffmpeg.org/ffmpeg-filters.html#rotate) for rotating the frames

You can use `ffplay` before you apply them using `ffmpeg`. For example:

```
ffplay -hide_banner -vf 'crop=in_w-1350:in_h-400:0, scale=640:-2' IN.mp4 
```

- `crop=in_w-1350:in_h-400:0`
  - `in_w` and `in_h` are the input width and height
  - crops 1350 pixels from the right
  - 200 pixels from the top
  - 200 pixels from the bottom
  - sets the view box's x coordinate to $x = 0$
  - Y coordinate is set automatically to $y = 200$ because the filter leaves the center of the frame as default.
- `scale=640:-2` scales down the width to 640 pixel while keeping the aspect ratio.

Running this command will play the video with the applied filters.

If the frames are [rotated](rotation-of-videos-on-smartphones) you can try to ignore rotation in the metadata by using `-noautorotate`:

```
ffplay -hide_banner -vf 'crop=in_w-1350:in_h-400:0, scale=640:-2' IN.mp4 -noautorotate
```

```{warning}
Currently [`ffplay -noautorotate` seems to have a bug](https://trac.ffmpeg.org/ticket/9822), so use this option only with `ffmpeg`
```

### Converting to WebM container

A video is typically delivered in a [container format](https://en.wikipedia.org/wiki/Container_format) which allows video, sounds for different languages, subtitles etc to be included in a single file. Browsers support various containers which in turn support many codecs. Mozilla Developer Network (MDN) has [recommendations which codec and container to choose](https://developer.mozilla.org/en-US/docs/Web/Media/Formats/Video_codecs#choosing_a_video_codec). According to MDN the first choice is:

```{note}
:class: margin
PDF and ZIP files are containers too.
```

- WebM container with
  - VP9 codec for everyday videos 
  - AV1 codec for high-quality videos

Examples using each codecs, WebM container and the filters that we used before follow:

#### AV1 in WebM container

```
ffmpeg -hide_banner \
	-i IN.mp4 \
	-vf 'crop=in_w-1350:in_h-400:0, scale=640:-2' \
	-c:v libaom-av1 \
	-b:v 0 \
	OUT.webm
```

- `c:v` selects encoding
- `-b:v 0` gives every frame freedom to take as many bits they want
- sound codec is automatically selected according to the container
- `-crf` is not explicitly set. So it is automatically set to a middle value like 32. `0` means lossless.

```{note}
Encoding using AV1 can take a long time. Starting an encoding job on a cloud computer and downloading it later can be beneficial. Using a terminal multiplexer like `tmux` which supports running a job in a background after logging out can help.
```

#### VP9 in WebM container

Refer to [VP9 two-pass encoding](https://trac.ffmpeg.org/wiki/Encode/VP9#twopass)

### Copying metadata

Metadata in a media file contains useful information like recording time, recording device and GPS coordinates. ffmpeg should automatically copy the metadata. You can check the metadata using `ffprobe`:

```bash
ffprobe OUT.webm
```
```
Input #0, matroska,webm, from 'OUT.webm':
  Metadata:
	...
    LOCATION        : +48.8318+012.9600/
    LOCATION-eng    : +48.8318+012.9600/
    COM.ANDROID.VERSION: 12
    ENCODER         : Lavf59.16.100
...
```

## Embedding

Markdown supports HTML and in HTML videos can be embedded using the [`<video>` tag](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/video).

In Sphinx, content which is not converted but directly copied to the output are typically placed to the `_static` directory. So we copy the converted video to this directory and then we embed the video using:

```html
<video controls>
	<source
		src="_static/xadc_ext_pin_reading_boolean_thumbwheel_potentiometer_demo.webm"
		type="video/webm"
	>
</video>
```
Output:

<video controls>
	<source
		src="_static/xadc_ext_pin_reading_boolean_thumbwheel_potentiometer_demo.webm"
		type="video/webm"
	>
</video>

Refer to [`<video>` tag at MDN](https://developer.mozilla.org/en-US/docs/Web/HTML/Element/video) for options.
