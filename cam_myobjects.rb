require 'camellia'
include Camellia

def cam_myobjects
  
  # Parameters 
  threshold = 50 
  displayed_model = 2

  # Constants
  model_images = ["edog5", "dvd", "mrpotato4"]
  test_images = ["scene1", "scene3", "scene4", "scene5", "scene7"]
  cx = [450, 390, 550]
  cy = [340, 510, 450]
  width = [720, 690, 500]
  height = [640, 910, 500]
  
  # Load model images
  models = []
  imodel = []
  (0...model_images.size).each do |i|
    filename = "resources/photos/#{model_images[i]}.bmp"
    puts "Feature point detection on #{model_images[i]} ..."
    image = CamImage.new
    imodel << image # Store this image
    image.load_bmp(filename)
    points = CamKeypoints.new(100000)
    points.id = i
    points.cx = cx[i]
    points.cy = cy[i]
    image.to_yuv.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
    models << points
  end

  # Define double image
  dimage = CamImage.new(imodel[0].width, imodel[0].height * 2, imodel[0].depth, CAM_CHANNELSEQ_RGB);
  roi = CamROI.new(0, 0, imodel[0].height, imodel[0].width, imodel[0].height)

  # Test all 150 images
  nb_features = 0
  score = 0

  (0...test_images.size).each do |i|
    filename = "resources/photos/#{test_images[i]}.bmp"
    puts "Feature point detection on #{test_images[i]}..."
    image = CamImage.new
    image.load_bmp(filename)
    points = CamKeypoints.new(100000)
    image.to_yuv.fast_hessian_detector(points, threshold, CAM_UPRIGHT)
    puts "# features = #{points.nb_points}"
    nb_features += points.nb_points

    # Create result image
    dimage.roi = nil
    imodel[displayed_model].copy_to dimage
    dimage.roi = roi
    image.copy_to dimage
    dimage.roi = nil

    (0...model_images.size).each do |j|
      matches = CamKeypointsMatches.new
      models[j].matching2(points, matches)   
      # Find affine parameters
      res = matches.find_affine_transform2
      score += matches.nb_matches - matches.nb_outliers
      if j == displayed_model
	# Draw lines between model and target
	matches.each do |m|
	  if m.mark != -1
	    m.p1.draw(dimage, cam_rgb(255, 0, 0))
            x1 = m.p1.x
            y1 = m.p1.y
            x2 = m.p2.x
            y2 = m.p2.y + image.height
            dimage.draw_line(x1, y1, x2, y2, cam_rgb(0, 255, 0))
	  end
	end
	matches.each do |m|
	  m.p2.y += image.height
          m.p2.draw(dimage, 128)
	end
      end

      xy = []; uv = []
      for c in 0..6 do 
	xy[c] = CamPoint.new
	uv[c] = CamPoint.new
      end
      # Draw box on model and target
      xy[0].x = xy[3].x = models[j].cx - width[j] / 2
      xy[0].y = xy[1].y = models[j].cy - height[j] / 2
      xy[1].x = xy[2].x = models[j].cx + width[j] / 2
      xy[2].y = xy[3].y = models[j].cy + height[j] / 2
      xy[4].x = xy[5].x = models[j].cx
      xy[4].y = xy[6].y = models[j].cy 
      xy[5].y = models[j].cy - height[j] / 4
      xy[6].x = models[j].cx + width[j] / 4
      for c in 0..6 do 
        uv[c] = xy[c].apply_affine_transform(res[0])
        uv[c].y += image.height
      end
      if j == displayed_model
	dimage.draw_line(xy[0].x, xy[0].y, xy[1].x, xy[1].y, 0)
	dimage.draw_line(xy[2].x, xy[2].y, xy[1].x, xy[1].y, 0)
	dimage.draw_line(xy[2].x, xy[2].y, xy[3].x, xy[3].y, 0)
	dimage.draw_line(xy[0].x, xy[0].y, xy[3].x, xy[3].y, 0)
	dimage.draw_line(xy[4].x, xy[4].y, xy[5].x, xy[5].y, 0)
	dimage.draw_line(xy[6].x, xy[6].y, xy[4].x, xy[4].y, 0)
      end
      dimage.draw_line(uv[0].x, uv[0].y, uv[1].x, uv[1].y, 0)
      dimage.draw_line(uv[2].x, uv[2].y, uv[1].x, uv[1].y, 0)
      dimage.draw_line(uv[2].x, uv[2].y, uv[3].x, uv[3].y, 0)
      dimage.draw_line(uv[0].x, uv[0].y, uv[3].x, uv[3].y, 0)
      dimage.draw_line(uv[4].x, uv[4].y, uv[5].x, uv[5].y, 0)
      dimage.draw_line(uv[6].x, uv[6].y, uv[4].x, uv[4].y, 0)
    end

    # Save result image
    dimage.save_bmp("output/#{test_images[i]}.bmp")	
  end

  puts "# features in scenes = #{nb_features}"
  puts "# matching features = #{score} (#{score * 100.0 / nb_features})"

end

cam_myobjects
