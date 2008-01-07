#!/usr/bin/env ruby

require 'ftools'
require 'find'
require 'rbconfig'

lib_dir = File.join(Config::CONFIG['sitedir'],
		    Config::CONFIG['MAJOR'] + "." + Config::CONFIG['MINOR'])

File.install('../lib/camellia-fox.rb', File.join(lib_dir,'camellia-fox.rb'), 0644, true)
File.install('../lib/camellia.dll', File.join(lib_dir,'camellia.dll'), 0644, true)
