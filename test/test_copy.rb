require 'test/unit'
require 'lib/camellia'
include Camellia
class TestCopy < Test::Unit::TestCase
  def test_copy
    source=CamImage.new
    # load picture alfa156.bmp
    source.load_bmp("resources/alfa156.bmp")
    dest=CamImage.new(source.width,source.height,CAM_DEPTH_8U,CAM_COLORMODEL_RGBA)
    source.copy(dest)
    dest.save_bmp("output/copy_alfa156.bmp")
  end
end
