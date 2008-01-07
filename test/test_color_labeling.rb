require 'test/unit'
require 'lib/camellia'
include Camellia
class TestLabeling < Test::Unit::TestCase
  def test_labeling

    # load picture alfa156.bmp
    image=CamImage.new
    image.load_bmp("resources/alfa156.bmp")
    yuv=image.to_yuv
    
    # set color clusters
    clusters=CamTable.new
    clusters.set([
      # Ymin Ymax Umin Umax Vmin Vmax
        0,   60,  0,   255, 0,   255, # Black
        230, 255, 0,   255, 0,   255, # White
        0,   255, 0,   255, 140, 255  # Red
    ])
    cluster_colors=[cam_rgb(0,0,0),cam_rgb(255,255,255),cam_rgb(255,0,0)]
    
    # threshold and encode
    encoded=yuv.encode_color(clusters)
    puts "Number of runs : #{encoded.nb_runs}"
    
    # labeling
    blobs=encoded.labeling!    
    puts "#{blobs.nb_blobs} blobs detected"
    
    # print and deaw the results 
    i=0
    blobs.each {|b|
      puts "Blob #{i} : Val=#{b.value} (#{b.top},#{b.left},#{b.width},#{b.height}) Surface=#{b.surface}"
      image.draw_rectangle(b.left,b.top,b.left+b.width-1,b.top+b.height-1,cluster_colors[b.value-1])
      i=i+1
    }
    
    # save the result
    image.save_bmp("output/ruby_alfa156_labeling.bmp")
    
    # assertions
    assert_equal(encoded.nb_runs,3026)
    assert_equal(blobs.nb_blobs,190)
  end
end
