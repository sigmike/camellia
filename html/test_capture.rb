require 'rubygems'
require_gem 'camellia'
include Camellia
    
image=CamImage.new
camera=CamCapture.new
i=0
loop do
  camera.capture image
  puts "Grabbed image #{i}..."
  image.save_bmp "output/capture#{i}.bmp"
  i+=1
end
