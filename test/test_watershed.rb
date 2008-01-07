require 'test/unit'
require 'lib/camellia'
include Camellia
class TestWatershed < Test::Unit::TestCase
  def test_watershed
    [["lsun",60],
     ["lsun_32x64e20-bord",50],
     ["lsun_48x80e20",40],
     ["lsun_56x88e20",50]].each do |fn,dyn|
      source=CamImage.new
      watershed=CamImage.new
      tob=CamTableOfBasins.new
      s="resources/#{fn}.pgm"
      puts "Watershed on #{s} :"
      source.load_pgm(s)
      source.hierarchical_watershed(watershed,tob)
      tob.each { |basin|
        tob.get_rid_of(basin) if basin.dynamics<dyn
      }
      tob.each_with_index { |basin,i|
        if basin.surface!=0 or basin.dynamics==CAM_NOT_COMPUTED
          print "Basin #{i} : Dynamics = #{basin.dynamics}, Minimum = #{basin.minimum}, Surface = #{basin.accsurface}, (x,y)=(#{basin.x},#{basin.y})"
          j=i
          while tob[j].dynamics!=CAM_NOT_COMPUTED
            printf "->#{tob[j].flooded}"
            j=tob[j].flooded
          end
          printf "\n"
        end
      }
      watershed.hierarchical_watershed_regions!(tob)
      watershed.save_pgm("output/#{fn}.pgm")
    end
  end
end
