#!/usr/local/bin/ruby

# rb2html
# Copyright (c) 2001, 2004 HORIKAWA Hisashi. All rights reserved.
#     http://www.nslabs.jp/
#     mailto:vzw00011@nifty.ne.jp

# Rubyスクリプト，Javaソース，C++ソースをHTMLに整形する
# キーワード，コメント，文字列に色を付ける

require File.expand_path('rb2html-sub.rb', File.dirname(__FILE__))

class Ruby2Html < Source2Html
  def initialize(formatter)
    super(formatter)

    # lex.c wordlist[]
    @keywords = Hash.new
    %w(
      end else case ensure module elsif def rescue not then yield for
      self false retry return true if defined? super undef break in do
      nil until unless or next when redo and begin __LINE__ class __FILE__
      END BEGIN while alias
    ).each {|k| @keywords[k.strip] = 1 }

    @here_end = Array.new
  end

  SEP_CHARS = {
    "(" => ")",
    "[" => "]",
    "{" => "}",
    "<" => ">"}

  def get_sep_char(ch)
    if SEP_CHARS[ch]
      return SEP_CHARS[ch]
    else
      return ch
    end
  end

  TO_STATE = {
    'w' => 9,  # 文字列の配列
    'q' => 3,  # シングルクォート文字列
    'Q' => 2,  # ダブルクォート文字列
    'r' => 4   # 正規表現
  }
  def state1(ch)
    # 地の文
    case ch
    when '"'
      que_out()
      @result << @fo.begin_literal << "&quot;"
      @sep_char = '"'
      @state = 2
      return
    when "'"  # シングルクォート文字列
      que_out()
      @result << @fo.begin_literal << "'"
      @sep_char = "'"
      @state = 3
      return
    when '/'
      if @line[0, 1] != ' '
        que_out()
        @result << @fo.begin_literal << "/"
        @sep_char = '/'
        @state = 4
        return
      end
    when '#'
      que_out()
      @result << @fo.begin_comment << "#" << html_escape(@line) << @fo.end_comment
      @line = ''
      return
    when '$'
      if @line[0, 1] == '\''
        @que << ch << @line.shift
        return
      end
    when '<'
      if @line[0, 1] == '<' && @line[1, 1] == '\''
        que_out()
        @result << html_escape("<<'")
        @line = @line[2, @line.length]
        
        tail = ''
        while /[a-zA-Z0-9]/ =~ @line[0, 1]
          tail << @line.shift
        end
        if tail != ''
          @here_end << [0, tail]
          @result << html_escape(tail)
        end
        @result << html_escape(@line.shift) # tailの後ろの'
        return
      elsif @line[0, 1] == '<' && @line[1, 1] != ' '
        que_out()
        @result << html_escape(ch + @line.shift)
        @result << html_escape(@line.shift) if @line[0, 1] == '"'
        
        tail = ''
        while /[a-zA-Z0-9]/ =~ @line[0, 1]
          tail << @line.shift
        end
        if tail != ''
          @here_end << [1, tail]
          @result << html_escape(tail)
        end
        @result << html_escape(@line.shift) if @line[0, 1] == '"'
        return
      end
    when '?'
      # ?/ => 数値リテラル
      if @line != ''
        @que << ch << @line.shift
        return
      end
    when '%'
      if TO_STATE[@line[0, 1]]
        @state = TO_STATE[@line[0, 1]]
        que_out()
        @result << @fo.begin_literal << html_escape(ch + @line[0, 2])
        @sep_char = get_sep_char(@line[1, 1])
        @line = @line[2, @line.length]
        return
      end
    end
    @que << ch
  end # state1()

  def state2(ch)
    # "..."
    case ch
    when @sep_char
      @result << html_escape(@que + ch) << @fo.end_literal
      @que = ''
      @state = 1
      return
    when '\\'
      if @line != ''
        @que << ch << @line.shift
        return
      end
    end
    @que << ch
  end

  def parse_line(s)
    @line = s
    @result << @fo.begin_literal if @state == 9 # %w(...)
  
    if @state == 1
      if @here_end.size > 0
        if @here_end[0][0] == 0
          # <<'EOF'
          @state = 6
        else
          # <<EOF
          @state = 7
        end
      elsif @line == '=begin'
        @result << @fo.begin_comment << html_escape(@line) << @fo.end_comment
        @state = 8
        return
      end
    end

    case @state
    when 6
      # <<'EOF'
      @result << @fo.begin_literal << html_escape(@line) << @fo.end_literal
      if @line == @here_end[0][1]
        @here_end.shift
        @state = 1
      end
      return
    when 7
      # <<EOF
      @result << @fo.begin_literal << html_escape(@line) << @fo.end_literal
      if @line == @here_end[0][1]
        @here_end.shift
        @state = 1
      end
      return
    when 8
      # =begin ... =end
      @result << @fo.begin_comment << html_escape(@line) << @fo.end_comment
      if @line == "=end"
        @state = 1
      end
      return
    end
    
    while @line != ''
      ch = @line.shift
      case @state
      when 1
        # 地の文
        state1(ch)
      when 2
        # "..."
        state2(ch)
      when 3
        # '...'
        case ch
        when @sep_char
          @result << html_escape(@que + ch) << @fo.end_literal
          @que = ''
          @state = 1
        when '\\'
          if @line[0, 1] == '\\' || @line[0, 1] == '\''
            @que << ch << @line.shift
          else
            @que << ch
          end
        else
          @que << ch
        end
      when 4
        # /.../
        case ch
        when @sep_char
          i = 0
          while @line[i, 1] != '' && "ioxmnesu".index(@line[i, 1])
            i += 1
          end
          @result << html_escape(@que + ch + @line[0, i]) << @fo.end_literal
          @line = @line[i, @line.length] if i > 0
          @que = ''
          @state = 1
        else
          @que << ch
        end
      when 9
        # %w(...)
        case ch
        when @sep_char
          @result << html_escape(@que + ch) << @fo.end_literal
          @que = ''
          @state = 1
        else
          @que << ch
        end
      else
        print "Internal error.\n"
        exit 1
      end
    end
    
    if @state == 1
      que_out()
    else
      @result << html_escape(@que)
      @que = ''
    end
    
    @result << @fo.end_literal if @state == 9
  end # parse_line()
end

if __FILE__ == $0
  f = Ruby2Html.new(HtmlFormatter.new)
  print f.format_io(ARGF)
end
