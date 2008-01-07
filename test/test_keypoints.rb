require 'test/unit'
require 'lib/camellia'
include Camellia
require 'rubygems'
require 'gnuplot'

class TestKeypoints < Test::Unit::TestCase

  def test_keypoints_scale

    puts "Keypoints detection :"
    image = CamImage.new(256, 256)
    points = CamKeypoints.new(1000) 
      
    # Draw 2 rectangles
    image.set!(0)
    image.draw_rectangle(102, 120, 156, 152, 255)
    image.fill_color(103, 121, 255)
    image.draw_rectangle(122, 35, 176, 67, 128)
    image.fill_color(123, 36, 128)

    # Draw 2 small squares
    xp = [-1, 1, 1, -1]
    yp = [-1, -1, 1, 1]
    angle = [20, 30]
    centerx = [192, 50]
    for j in 0..1 do
      costheta = Math.cos(angle[j] * 2 * Math::PI / 360)
      sintheta = Math.sin(angle[j] * 2 * Math::PI / 360)
      p = []
      for i in 0..3 do
	p[i] = CamPoint.new( ((costheta * xp[i] - sintheta * yp[i]) * 15 + centerx[j]).to_i,
			     ((sintheta * xp[i] + costheta * yp[i]) * 15 + 192).to_i )
      end
      for i in 0..3 do
	image.draw_line(p[i].x, p[i].y, p[(i+1) % 4].x, p[(i+1) % 4].y, 255)
      end
      image.fill_color(centerx[j], 192, 255) 
    end

    image.fast_hessian_detector(points, 2000);
    points.each do |point|
      puts "x=#{point.x} y=#{point.y} value=#{point.value} scale=#{point.scale} size=#{point.size} angle=#{point.angle}"
    end	
    image.draw_keypoints!(points, 128);
    image.save_pgm("output/features_reference.pgm");

  end
 
  def test_keypoints_yale_faces
   
    # Parameters 
    threshold = 100 

    # Load model images
    models = []
    (1..15).each do |i|
      filename = "resources/yalefaces/subject#{'%02d' % i}.normal.pgm"
      puts "Keypoint detection on #{filename} ..."
      image = CamImage.new
      image.load_pgm(filename)
      points = CamKeypoints.new(10000)
      points.id = i
      image.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
      image.draw_keypoints!(points, 128)
      image.save_pgm("output/yalefaces#{'%02d' % i}.pgm")
      models << points
    end
    
    # Compile a kdTree
    kdTree = CamKeypointsKdTree.new(models)

    # Test all 150 images
    feature = ["centerlight", "glasses", "happy", "leftlight", "noglasses", "rightlight", "sad", "sleepy", "surprised", "wink"]
    nb_features = 0
    rate = 0
    nb_good_matches = 0
    matches = CamKeypointsMatches.new

    (1..15).each do |i|
      (1..10).each do |j|
	filename = "resources/yalefaces/subject#{'%02d' % i}.#{feature[j-1]}.pgm"
	puts "Keypoint detection on #{filename} ..."
	image = CamImage.new
	image.load_pgm(filename)
	points = CamKeypoints.new(10000)
	image.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
	puts "# features = #{points.nb_points}"
	nb_features += points.nb_points
	#best_match = points.matching(models, matches)
	best_match = points.matchingKdTree(kdTree, matches, 100)
	puts "Best match is : #{best_match} (#{matches.nb_matches})"
	if best_match  == i then
	  rate += 1
	  nb_good_matches += matches.nb_matches
	end
      end
    end

    puts "Recognition rate is #{rate * 100.0 / (15*10)}"
    puts "Good matches = #{nb_good_matches}"
    puts "Nb features = #{nb_features} (% matching = #{nb_good_matches * 100.0 / nb_features}%)"

  end

end
