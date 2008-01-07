require 'test/unit'
require 'lib/camellia'
include Camellia
class TestHough < Test::Unit::TestCase
  def test_hough
    image=CamImage.new(100,100)
    # Draw a filled circle in mask
    image.set!(0)
    image.draw_circle(40,40,50,128)
    image.fill_color(40,40,128)
    res=image.hough_circle(10,40,60)
    puts "Found a circle at (#{res[1]},#{res[2]}) (radius=#{res[3]}) (confidence=#{res[0]})"
    assert((res[1]-40).abs<=1)
    assert((res[2]-40).abs<=1)
   
    for i in 1..3 do
    	# Load an image with a road sign
    	image.load_bmp("resources/road#{i}.bmp")
    	yuv=CamImage.new
	image.to_yuv(yuv)
	yuv.set_roi(CamROI.new(1,8,8,image.width-16,image.height-128))
	yuv.fixed_filter!(CAM_GAUSSIAN_3x3)
	res=yuv.hough_circle(100,6,25)
    	image.draw_circle(res[1],res[2],res[3],cam_rgb(255,0,0))
    	image.save_bmp("output/road_circle_hough#{i}.bmp")
   	puts "Found a circle at (#{res[1]},#{res[2]}) (radius=#{res[3]}) (confidence=#{res[0]})"
    end
  end
end
