require 'test/unit'
require 'lib/camellia'
include Camellia

class TestCapture < Test::Unit::TestCase
  def test_capture
    image=CamImage.new
    camera=CamCapture2.new
    for i in 1..30 do
      camera.capture(image)
      puts "Grabbed image #{i}"
    end
    image.save_bmp('output/capture.bmp')
  end
end
