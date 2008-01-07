require 'rubygems'
require_gem 'camellia'
include Camellia
    
source=CamImage.new

# load picture chess.pgm
source.load_pgm("resources/chess.pgm")
mask=CamImage.new(source.width,source.height)

# draw a filled circle in mask
mask.set!(0)
mask.draw_circle(mask.width/2,mask.height/2,50,255)
mask.fill_color(mask.width/2,mask.height/2,255)

# encode the mask and associate it to the source image
encoded_mask=mask.encode
source.mask=encoded_mask
encoded_mask.inverse!
dest=CamImage.new
source.copy(dest) # copies only the mask! not like dup or clone!

# and then compute a sobel inside
encoded_mask.inverse!
source.sobel_v_abs(dest) # sobel only on the mask 
dest.save_pgm("output/ruby_chess_sobel_mask.pgm")
