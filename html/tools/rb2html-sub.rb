
# rb2html
# Copyright (c) 2001, 2004 HORIKAWA Hisashi. All rights reserved.
#     http://www.nslabs.jp/
#     mailto:vzw00011@nifty.ne.jp

def html_escape(s)
  # エスケープする
  if s
    r = s.dup
    r.gsub! '&', '&amp;'
    r.gsub! '"', '&quot;'
    r.gsub! '<', '&lt;'
    r.gsub! '>', '&gt;'
    r
  else
    nil
  end
end

class String
  def shift
    r = self[0, 1]
    self[0, 1] = ''
    return r
  end
end

class Source2Html
  def initialize(formatter)
    @que = ''
    @line = ''
    @state = 1
    @filename = ''
    @fo = formatter
  end

  def que_out()
    # 地の文の出力, キーワードの色付け
    while @que =~ /[a-zA-Z_][a-zA-Z0-9_]*[?!]?/
      @que = $'
      if @keywords[$&] && $`[-1] != ?.
        @result << html_escape($`) << @fo.begin_keyword << $& << @fo.end_keyword
      else
        @result << html_escape($`) << $&
      end
    end
    @result << html_escape(@que)
    @que = ''
  end

  def parse(fp)
    lno = 0
    while line = fp.gets
      lno += 1
      @result << sprintf("%3d| ", lno)
      line.gsub! /[\r\n]/, ''
      parse_line(line)
      @result << "\n"
    end
  end # def
  
  def format_io(io)
    @filename = io.path

    @result = <<EOF
<html>
<head>
  <title>#{@filename}</title>
<style type="text/css">
.source .literal { color:#660066; }
.source .comment { color:green; }
.source .keyword { color:blue; }
.source .preprocessor { color:purple; }
</style>
</head>
<body>
EOF
    @result << @fo.start_formatted(@filename)
    parse(io)
    @result << @fo.end_formatted()
    @result << <<EOF
</body>
</html>
EOF
    return @result
  end

  def format_string(str)
    format_io(StringIO(str))
  end
end

class HtmlFormatter
  def start_formatted(filename)
    return <<EOF
<div style="color:blue;margin-top:1em;font-family:sans-serif">#{filename}</div>
<pre class="source">
EOF
  end
  def end_formatted()
    '</pre>'
  end
  def begin_literal
    '<span class="literal">' # color:#660066
  end
  def end_literal
    '</span>'
  end
  def begin_comment
    '<span class="comment">' # color:green
  end
  def end_comment
    '</span>'
  end
  def begin_keyword
    '<span class="keyword">' # color:blue
  end
  def end_keyword
    '</span>'
  end

  def begin_preprocessor
    '<span class="preprocessor">'
  end
  def end_preprocessor
    '</span>'
  end
end

class EWBFormatter
  def begin_literal
    ''
  end
  def end_literal
    ''
  end
  def begin_comment
    ''
  end
  def end_comment
    ''
  end
  def begin_keyword
    ''
  end
  def end_keyword
    ''
  end
end
