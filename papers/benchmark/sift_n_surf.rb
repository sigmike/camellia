def sift_detector(image)
  # Save the image
  image.save_raw_pgm('tmp.pgm')
  start = Time.now
  # Call sift program
  `./sift <tmp.pgm >sift.txt`
  stop = Time.now
  # Open the result file
  str = IO.read('sift.txt')
  arr = str.split(/\s+/)
  # Retrieve the number of sift points
  nb = arr.shift.to_i
  size = arr.shift.to_i
  #puts "descriptor size is #{size}"
  points = CamKeypoints.new(nb);
  # Retrieve all sift points
  for i in 1..nb do
    p = CamKeypoint.new
    p.y = arr.shift.to_i
    p.x = arr.shift.to_i
    p.scale = (arr.shift.to_f * 8).to_i
    p.angle = (arr.shift.to_f * 360 / Math::PI).to_i
    p.descriptor = arr.slice!(0, size).map {|x| x.to_i}
    points.add p
  end 
  # Remove temp files
  File.delete('tmp.pgm')
  File.delete('sift.txt')
  return [points, stop - start]
end

def surf_detector(image, options = "")
  # Save the image
  image.save_raw_pgm('tmp.pgm')
  start = Time.now
  # Call sift program
  `./surf.ln -i tmp.pgm -o surf.txt #{options}`
  stop = Time.now
  # Open the result file
  str = IO.read('surf.txt')
  arr = str.split(" ")
  # Retrieve the number of surf points
  size = arr.shift.to_i - 1
  #puts "descriptor size is #{size}"
  nb = arr.shift.to_i
  points = CamKeypoints.new(nb);
  # Retrieve all surf points
  for i in 1..nb do
    p = CamKeypoint.new
    p.x = arr.shift.to_i
    p.y = arr.shift.to_i
    p.scale = (Math::sqrt(1 / arr.shift.to_f) * 4).to_i
    arr.shift
    arr.shift
    arr.shift
    p.angle = 0
    p.descriptor = arr.slice!(0, size).map {|x| (x.to_f * 10000).to_i}
    points.add p
  end 
  # Remove temp files
  File.delete('tmp.pgm')
  File.delete('surf.txt')
  return [points, stop - start]
end


