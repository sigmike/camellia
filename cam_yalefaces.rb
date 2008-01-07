require 'camellia'
include Camellia

def cam_yalefaces
  
  # Parameters 
  threshold = 0 

  # Constants
  cx = [181, 195, 164, 193, 189, 118, 162, 161, 155, 171, 183, 208, 191, 133, 168]
  cy = [138, 158, 165, 150, 137, 145, 164, 172, 146, 143, 148, 147, 164, 152, 165]
  width = 120; height = 140  
  
  # Load model images
  models = []
  imodel = []
  (1..15).each do |i|
    filename = "resources/yalefaces/subject#{'%02d' % i}.normal.pgm"
    puts "Feature point detection on #{filename} ..."
    image = CamImage.new
    imodel << image # Store this image
    image.load_pgm(filename)
    points = CamKeypoints.new(10000)
    points.id = i
    points.cx = cx[i-1]
    points.cy = cy[i-1]
    image.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
    models << points
  end

  # Define double image
  dimage = CamImage.new(imodel[0].width, imodel[0].height * 2, imodel[0].depth);
  roi = CamROI.new(0, 0, imodel[0].height, imodel[0].width, imodel[0].height)

  # Test all 150 images
  feature = ["centerlight", "glasses", "happy", "leftlight", "noglasses", "rightlight", "sad", "sleepy", "surprised", "wink"]
  nb_features = 0
  rate = 0
  nb_good_matches = 0

  (1..15).each do |i|
    (1..10).each do |j|
      filename = "resources/yalefaces/subject#{'%02d' % i}.#{feature[j-1]}.pgm"
      puts "Feature point detection on #{filename}..."
      image = CamImage.new
      image.load_pgm(filename)
      points = CamKeypoints.new(10000)
      image.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
      puts "# features = #{points.nb_points}"
      nb_features += points.nb_points
      matches = CamKeypointsMatches.new
      best_match = points.matching(models, matches)
      puts "Best match is : #{best_match} (#{matches.nb_matches})"
      if best_match == i then
	rate += 1
	nb_good_matches += matches.nb_matches

	# Create result image
	dimage.roi = nil
	imodel[i-1].copy_to dimage
	dimage.roi = roi
	image.copy_to dimage
	dimage.roi = nil

	# Draw lines between model and target
	k = 0
	matches.each do |m|
	  m.p1.draw(dimage, 128)
	  x1 = m.p1.x
	  y1 = m.p1.y
	  x2 = m.p2.x
	  y2 = m.p2.y + image.height
	  dimage.draw_line(x1, y1, x2, y2, 128)
	end

	# Find affine parameters
	res = matches.find_affine_transform
	xy = []; uv = []
	for c in 0..6 do 
	  xy[c] = CamPoint.new
	  uv[c] = CamPoint.new
	end
	# Draw points below
	matches.each do |m|
	  m.p2.y += image.height
	  m.p2.draw(dimage, 128)
	end
	# Draw box on model and target
	xy[0].x = xy[3].x = models[i - 1].cx - width / 2
	xy[0].y = xy[1].y = models[i - 1].cy - height / 2
	xy[1].x = xy[2].x = models[i - 1].cx + width / 2
	xy[2].y = xy[3].y = models[i - 1].cy + height / 2
	xy[4].x = xy[5].x = models[i - 1].cx
	xy[4].y = xy[6].y = models[i - 1].cy
	xy[5].y = models[i - 1].cy - height / 4
	xy[6].x = models[i - 1].cx + width / 4
	for c in 0..6 do 
	  uv[c] = xy[c].apply_affine_transform(res[0])
	  uv[c].y += image.height
	end
	dimage.draw_line(xy[0].x, xy[0].y, xy[1].x, xy[1].y, 0)
	dimage.draw_line(xy[2].x, xy[2].y, xy[1].x, xy[1].y, 0)
	dimage.draw_line(xy[2].x, xy[2].y, xy[3].x, xy[3].y, 0)
	dimage.draw_line(xy[0].x, xy[0].y, xy[3].x, xy[3].y, 0)
	dimage.draw_line(xy[4].x, xy[4].y, xy[5].x, xy[5].y, 0)
	dimage.draw_line(xy[6].x, xy[6].y, xy[4].x, xy[4].y, 0)
	dimage.draw_line(uv[0].x, uv[0].y, uv[1].x, uv[1].y, 0)
	dimage.draw_line(uv[2].x, uv[2].y, uv[1].x, uv[1].y, 0)
	dimage.draw_line(uv[2].x, uv[2].y, uv[3].x, uv[3].y, 0)
	dimage.draw_line(uv[0].x, uv[0].y, uv[3].x, uv[3].y, 0)
	dimage.draw_line(uv[4].x, uv[4].y, uv[5].x, uv[5].y, 0)
	dimage.draw_line(uv[6].x, uv[6].y, uv[4].x, uv[4].y, 0)
	
	# Save result image
	dimage.save_pgm("output/yalefaces#{'%02d' % i}#{feature[j-1]}.pgm")	
      end
    end
  end

  puts "Recognition rate is #{rate * 100.0 / (15*10)}"
  puts "Good matches = #{nb_good_matches}"
  puts "Nb features = #{nb_features} (% matching = #{nb_good_matches * 100.0 / nb_features}%)"

end

cam_yalefaces
