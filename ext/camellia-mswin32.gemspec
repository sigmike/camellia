require 'rubygems'

spec = Gem::Specification.new do |s|
  s.name = "camellia"
  s.version = "2.7.0"
  s.author = "Bruno STEUX"
  s.email = "bruno.steux@ensmp.fr"
  s.homepage = "http://camellia.sourceforge.net"
  s.platform = Gem::Platform::CURRENT
  s.summary = "Image Processing & Computer Vision library"
  candidates = Dir.glob("{doc,lib,test,ext,inc}/**/*")
  s.files = candidates.delete_if do |item|
    item.include?(".svn")||item.include?(".lib")||item.include?(".ilk")||item.include?(".o")||item.include?(".gem")||item.include?(".so")
  end
  s.require_path = "lib"
  s.autorequire = "camellia"
  s.test_files = Dir.glob('test/*.rb')
  s.has_rdoc = false
  s.extra_rdoc_files = ["README"]
end

