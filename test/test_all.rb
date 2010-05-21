Dir["test/test_*.rb"].each do |file|
  next if File.basename(file) == "test_helper.rb"
  begin
    require file
  rescue Exception => e
    STDERR.puts "Could not load #{file}: #{e.message}"
  end
end
