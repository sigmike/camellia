require 'mkmf'
$LIBS << " -lstdc++"
$LIBS << " -lCamellia"
create_makefile('camellia')
