require 'camellia'
include Camellia

j=0
Thread.new do
    loop do
        sleep 0.01
        puts "#{j}..."
        j+=1
    end
end

# CamCapture2 is the original version from the C library
# It is not Ruby-threads compatible
image=CamImage.new
capture=CamCapture2.new(0)
for i in 1..10 do
    capture.capture image
    puts "Grabbed image #{i}...#{image.width}*#{image.height}"
end

# CamCapture is the Ruby specific version
# It is fully Ruby-threads compatible
capture=CamCapture.new(0)
puts "#{capture.width}*#{capture.height}"
image=CamImage.new
for i in 1..10 do
    capture.capture image
    puts "Grabbed image #{i}...#{image.width}*#{image.height}"
end

