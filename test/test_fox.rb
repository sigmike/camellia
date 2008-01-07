require 'rubygems'
require_gem 'fxruby'
include Fox

require_gem 'camellia'
require 'camellia-fox.rb'
include Camellia

class ImageWindow < FXMainWindow

  def initialize(app)
    # Invoke base class initializer first
    super(app, "Camellia/FXRuby test", nil, nil, DECOR_ALL, 0, 0, 440, 240)

    contents = FXHorizontalFrame.new(self,
      LAYOUT_SIDE_TOP|LAYOUT_FILL_X|LAYOUT_FILL_Y, 0, 0, 0, 0, 0, 0, 0, 0)

    # RIGHT pane for the buttons
    buttonFrame = FXVerticalFrame.new(contents, (FRAME_SUNKEN|LAYOUT_FILL_Y|
      LAYOUT_TOP|LAYOUT_LEFT), 0, 0, 0, 0, 10, 10, 10, 10)

    # Label above the buttons  
    FXLabel.new(buttonFrame, "Button Frame", nil,
      JUSTIFY_CENTER_X|LAYOUT_FILL_X);
    
    # Horizontal divider line
    FXHorizontalSeparator.new(buttonFrame, SEPARATOR_RIDGE|LAYOUT_FILL_X)

    # Erode button
    erodeBtn = FXButton.new(buttonFrame,
      "Erode Image...\tGrey scale erosion",
      nil, nil, 0, (FRAME_THICK|FRAME_RAISED|LAYOUT_FILL_X|
      LAYOUT_TOP|LAYOUT_LEFT), 0, 0, 0, 0, 10, 10, 5, 5) { |button|
      button.connect(SEL_COMMAND, method(:onCmdErode))
    }
    
    # Save button
    saveBtn = FXButton.new(buttonFrame,
      "Save Image...\tRead back image and save to file",
      nil, nil, 0, (FRAME_THICK|FRAME_RAISED|LAYOUT_FILL_X|
      LAYOUT_TOP|LAYOUT_LEFT), 0, 0, 0, 0, 10, 10, 5, 5)
    saveBtn.connect(SEL_COMMAND, method(:onCmdRestore))
    
    # Exit button
    FXButton.new(buttonFrame, "E&xit\tQuit FXRuby test", nil,
      getApp(), FXApp::ID_QUIT, (FRAME_THICK|FRAME_RAISED|LAYOUT_FILL_X|
      LAYOUT_TOP|LAYOUT_LEFT), 0, 0, 0, 0, 10, 10, 5, 5)
  
    # Create a FXBMPImage and load a picture from disk 
    @picture=FXBMPImage.new(getApp(), nil, IMAGE_KEEP|IMAGE_OWNED, 320, 240);
    FXFileStream.open('resources/alfa156.bmp', FXStreamLoad) { |stream| @picture.loadPixels(stream) }
    @picture.render
    source=@picture.to_camellia
    @image=source.to_yuv
    @image.set_roi(CamROI.new(@image,1)) # Select Y plane (B&W)

    @imgFrame=FXImageFrame.new(contents,@picture)

    # Make a tip
    FXToolTip.new(getApp())
  end

  # Create and initialize
  def create
    # Create the windows
    super

    # Make the main window appear
    show(PLACEMENT_SCREEN)
  end

  # Restore image from offscreen pixmap
  def onCmdRestore(sender, sel, ptr)
    saveDialog = FXFileDialog.new(self, "Save as BMP")
    if saveDialog.execute != 0
      @picture.restore
      # Get back a CamImage and save it
      @picture.to_camellia.save_bmp(saveDialog.filename)  
    end
    return 1
  end
  
  # Erode image
  def onCmdErode(sender, sel, ptr)
    @image.erode_circle5!
    @image.to_fox(getApp(),@picture)
    @picture.render
    @imgFrame.update
  end
  
end

if __FILE__ == $0
  # Make application
  application = FXApp.new("Camellia/FXRuby", "Camellia/FXRuby test")

  # Make the main window
  ImageWindow.new(application)

  # Create the application window and resources
  application.create

  # Run the application
  application.run
end

