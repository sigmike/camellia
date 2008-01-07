require 'test/unit'
require 'lib/camellia'
include Camellia

class TestWarping < Test::Unit::TestCase

  def test_warping
    source=CamImage.new
    # load picture chess.pgm
    source.load_pgm("resources/chess.pgm")
    resampled = CamImage.new(source.width / 2, source.height / 2, source.depth)
    warped = CamImage.new(source.width, source.height, source.depth)
    rotated = CamImage.new(source.width, source.height, source.depth)

    for i in 0..1 do
      interpolation = (i == 1) ? CAM_BILINEAR_INTERPOLATION : CAM_NN_INTERPOLATION
      
      p = []
      p << CamPoint.new(0, 0)
      p << CamPoint.new( (source.width << 16) - 1, 0)
      p << CamPoint.new( (source.width << 16) - 1, (source.height << 16) - 1)
      p << CamPoint.new(0, (source.height << 16) -1)
      source.warping(resampled, interpolation, false, *p)

      p = []
      p << CamPoint.new( ((source.width << 16) - 1) * 1 / 3, 0)
      p << CamPoint.new( ((source.width << 16) - 1) * 2 / 3, 0)
      p << CamPoint.new( (source.width << 16) - 1, (source.height << 16) - 1)
      p << CamPoint.new(0, (source.height << 16) -1)
      source.warping(warped, interpolation, true, *p)

      p = []
      p << CamPoint.new( (source.width << 16) / 2, -(source.height << 16) / 2)
      p << CamPoint.new( 3 * (source.width << 16) / 2, (source.height << 16) / 2)
      p << CamPoint.new( (source.width << 16) / 2, 3 * (source.height << 16) / 2)
      p << CamPoint.new( -(source.width << 16) / 2, (source.height << 16) / 2)
      source.warping(rotated, interpolation, true, *p)
    
      # save images
      resampled.save_pgm("output/chess_resampled_" + ((i == 1)?"bilinear":"nn") + ".pgm")
      warped.save_pgm("output/chess_warped_" + ((i == 1)?"bilinear":"nn") + ".pgm")
      rotated.save_pgm("output/chess_rotated_" + ((i == 1)?"bilinear":"nn") + ".pgm")
    end
  end

end
