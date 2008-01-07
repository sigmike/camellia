require 'test/unit'
require 'lib/camellia'
include Camellia
class TestArithm < Test::Unit::TestCase
  def test_arithm
    source=CamImage.new
    # load picture chess.pgm
    source.load_pgm("resources/chess.pgm")
    n=source.arithm(source,source,CAM_SUB)
    # resulting pixel sum should be 0
    assert_equal(n,0)
  end
end
