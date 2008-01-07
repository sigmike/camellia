require 'test/unit'
require 'lib/camellia'
include Camellia

class TestDraw < Test::Unit::TestCase
  def test_draw
    image=CamImage.new(320,240,CAM_DEPTH_8U,CAM_COLORMODEL_RGB)
    font=CamBitmapFont.new('resources/fonts/xenon2.bmp')
    image.set!(0)
    image.draw_line(0,0,320,240,cam_rgb(255,0,0))
    image.fill_color(50,100,cam_rgb(255,0,0))
    image.draw_text_bitmap('Hello World',100,140,font)
    image.save_bmp('output/drawing_ruby.bmp')
  end
end
