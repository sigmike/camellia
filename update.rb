puts <<EOS
CamelliaLib v2.5.0+ converter.
(c) 2006 Ecole des Mines de Paris
EOS
types=['ellia','Image','ROI','RLEImage','LinearFilterKernel',
  'MorphoMathsKernel','Table','MeasuresResults','SepFilterKernel',
  'Point','Run','BlobInfo','Blobs','WarpingParams',
  'ArithmParams','LabellingResults','LabelingResults',
  'MorphoMathsParams','TableOfBasins','MotionEstimation3DRSParams',
  'LUT','Histo','BlobAnalysisResults','Sobel3x3','SobelAbs3x3','Config',
  'MotionEstimation3DRSResults']
ARGV.each do |filename|
  puts "Processing #{filename}..."
  lines=IO.readlines(filename)
  file=File.new(filename,"w")
  lines.each do |line|
  str=line
    while (str=~/(.*?)Cam(\w+)(.*)/)!=nil
      if (!types.detect {|x| $2==x})
        file.print "#{$1}cam#{$2}"
      else
        file.print "#{$1}Cam#{$2}"
      end
      str=$3
    end
    file.puts str
  end
end
