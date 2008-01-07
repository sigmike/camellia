require 'rubygems'
require_gem 'fxruby'

require 'camellia'
include Camellia

class Fox::FXImage
  def to_camellia
    s=FXStream.new
    mem=" "*(width*height*4)
    s.open(FXStreamSave, width*height*4, mem)
    unbound_savePixels=FXImage.instance_method(:savePixels)
    bound_savePixels=unbound_savePixels.bind(self)
    bound_savePixels.call(s)
    image=CamImage.new(width, height, CAM_DEPTH_8U, CAM_COLORMODEL_RGBA)
    image.set_pixels(mem)
    return image
  end
end

class Camellia::CamImage
  def to_fox(app, picture=nil)
    if picture==nil
      pict=FXImage.new(app, nil, IMAGE_OWNED|IMAGE_KEEP, width, height)
    else
      pict=picture
    end if
    image=CamImage.new(width, height, CAM_DEPTH_8U, CAM_COLORMODEL_RGBA)
    copy(image)
    FXStream.new { |stream|
      stream.open(FXStreamLoad, image.imageSize, image.to_s)
      unbound_loadPixels=FXImage.instance_method(:loadPixels)
      bound_loadPixels=unbound_loadPixels.bind(pict)
      bound_loadPixels.call(stream)
    }
    return pict
  end
end

