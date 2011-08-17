.. _ecto_opencv.highgui:

Highgui Cells
==============
Common opencv image input and out, plus display.


Tricks
------

Saving images on a key stroke
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you attach the ImageSaver cell up to your graph you can trigger png saves
with the a trigger key in the following manner:

.. code-block:: python

  import ecto
  from ecto_opencv import highgui
  
  plasm = ecto.Plasm()
  video_display = highgui.imshow('imshow',
                                 name='video_cap', waitKey=10)
  imgsaver = highgui.ImageSaver()
  
  # ... make more cells here
  
  plasm.connect( # ... fill in graph
                some_image_source['image'] >> (video_display['input'], imgsaver['image']),
                video_display['out'] >> imgsaver['trigger'],
                )

On every frame, the video_display cell will output a user key, if any have been pressed,
and if ``s`` is the value, a png image will be saved.

Reference
---------
.. ectomodule:: ecto_opencv.highgui
