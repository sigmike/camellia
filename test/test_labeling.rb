require 'test/unit'
require 'lib/camellia'
include Camellia
class TestLabeling < Test::Unit::TestCase
  def test_labeling
    image=CamImage.new
    # load picture alfa156.bmp
    image.load_bmp("resources/alfa156.bmp")
    yuv=image.to_yuv
    # consider only the V plane (red)
    yuv.set_roi(CamROI.new(yuv,3)) 
    # threshold and encode
    thr=yuv.encode_threshold(150)
    # labeling
    blobs=thr.labeling!
    puts "#{blobs.nb_blobs} blobs detected"
    # draw rectangle on all detected blobs 
    blobs.each {|b| image.draw_rectangle(b.left,b.top,b.left+b.width-1,b.top+b.height-1,cam_rgb(255,0,0))}
    # save the resulting picture
    image.save_bmp("output/ruby_alfa156_labeling.bmp")
    # find out the biggest blob
    sorted=blobs.sort {|a,b| b.surface<=>a.surface}
    puts "The bigger blob is at position (#{sorted[0].cx},#{sorted[0].cy}) and its surface is #{sorted[0].surface}"
    # assertions
    assert_equal(blobs.nb_blobs,12)
    assert_equal(sorted[0].cx,170)
    assert_equal(sorted[0].cy,128)
    assert_equal(sorted[0].surface,601)
  end
end
