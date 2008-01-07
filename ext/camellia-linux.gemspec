require 'rubygems'

spec = Gem::Specification.new do |s|
  s.name = "camellia"
  CAMELLIA_VERSION = "2.7.0"
  s.version = CAMELLIA_VERSION
  s.author = "Bruno STEUX"
  s.email = "bruno.steux@ensmp.fr"
  s.homepage = "http://camellia.sourceforge.net"
  s.platform = Gem::Platform::CURRENT
  s.summary = "Image Processing & Computer Vision library"
  candidates = Dir.glob("{doc,lib,test,ext,inc}/**/*")
  candidates.delete_if do |item|
    item.include?(".svn") || item.include?(".lib") || item.include?(".ilk") || item.include?(".o") || item.include?(".gem") || item.include?(".so")
  end
  candidates << "CamelliaLib-" + CAMELLIA_VERSION + ".tar.gz"
  s.files = candidates
  s.require_path = "lib"
  s.autorequire = "camellia"
  s.test_files = Dir.glob('test/*.rb')
  s.has_rdoc = false
  s.extra_rdoc_files = ["README"]
  s.extensions=["ext/extconf.rb"]
end

if $0 == __FILE__
  Gem::manage_gems
  Gem::Builder.new(spec).build
end

