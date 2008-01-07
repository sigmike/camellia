require 'camellia'
include Camellia
require 'rubygems'
require 'gnuplot'

require 'sift_n_surf.rb'

def bench_square

  # Test on 50 differents images of squares
  image = CamImage.new(256, 256)
  x = [[], [], []]; y = [[], [], []]
  algorithm_names = ['CamKeypoints', 'sift', 'surf']

  for c in 1..50 do

    # Draw a square
    puts "Image \##{c}"
    for algorithm in 0..2 do
      puts "Testing with #{algorithm_names[algorithm]}..."

      image.set!(0)
      image.draw_rectangle(128 - c, 128 - c, 128 + c, 128 + c, 255)
      image.fill_color(128, 128, 255)

      case algorithm 
      when 0:
        points = CamKeypoints.new(1000) 
        start = Time.now
	image.fast_hessian_detector(points, 1000)
        stop = Time.now
	duration = stop - start
	puts "#{points.nb_points} points found. #{duration}s"
      when 1:
        points, duration = sift_detector(image)
        puts "#{points.nb_points} points found. #{duration}s"
      when 2:
	points, duration = surf_detector(image)
        puts "#{points.nb_points} points found. #{duration}s"
      end

      points.each do |point|
        #puts "x=#{point.x} y=#{point.y} value=#{point.value} scale=#{point.scale} size=#{point.size}"
        if (point.x - 128).abs <= c/3 and (point.y - 128).abs <= c/3 then
          #puts "Scale = #{point.scale}"
          x[algorithm] << (1 + c * 2)
          y[algorithm] << point.scale
        end
      end

      #Save result image
      image.draw_keypoints!(points, 128);
      image.save_pgm("features_squares_#{algorithm_names[algorithm]}_#{c}.pgm");
    end
  end

  # Draw the result with Gnuplot
  Gnuplot.open do |gp|
    Gnuplot::Plot.new( gp ) do |plot|

      plot.title  "Feature Points Scale Interpolation Test"
      plot.terminal "png size 640,480"
      plot.output "features_scale_interpolation.png"
      plot.ylabel "Feature point scale"
      plot.xlabel "Square size"

      for algorithm in 0..2 do
	plot.data << Gnuplot::DataSet.new( [x[algorithm], y[algorithm]] ) do |ds|
	  ds.with = "linespoints"
	  ds.title = algorithm_names[algorithm]
	end
      end
    end
  end 

end

bench_square
