require 'test/unit'
require 'lib/camellia'
include Camellia
class TestHistogram < Test::Unit::TestCase
  def test_histogram
    source=CamImage.new
    # load picture alfa156.bmp
    source.load_bmp("resources/alfa156.bmp")
    # convert to YUV
    yuv=CamImage.new
    yuv=source.to_yuv
    # compute the histogram
    result=CamImage.new
    yuv.histogram_2_channels(2,3,result)
    # save the result
    result.save_pgm("output/alfa156_histo.pgm")
  end
end
