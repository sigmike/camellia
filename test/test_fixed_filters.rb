require 'test/unit'
require 'lib/camellia'
include Camellia
class TestFixedFilters < Test::Unit::TestCase
  def test_fixed_filters
    source=CamImage.new
    result=CamImage.new
    # load picture chess.pgm
    source.load_pgm("resources/chess.pgm")
    # gaussian filtering 3x3
    source.fixed_filter(result,CAM_GAUSSIAN_3x3);
    # save the result
    result.save_pgm("output/ruby_chess_gaussian_3x3.pgm");
    # gaussian filtering 5x5
    source.fixed_filter(result,CAM_GAUSSIAN_5x5);
    # save the result
    result.save_pgm("output/ruby_chess_gaussian_5x5.pgm");
    # gaussian filtering 7x7
    source.fixed_filter(result,CAM_GAUSSIAN_7x7);
    # save the result
    result.save_pgm("output/ruby_chess_gaussian_7x7.pgm");
  end
end
