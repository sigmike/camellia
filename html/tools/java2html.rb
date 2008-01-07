#!/usr/local/bin/ruby

# rb2html
# Copyright (c) 2001, 2004 HORIKAWA Hisashi. All rights reserved.
#     http://www.nslabs.jp/
#     mailto:vzw00011@nifty.ne.jp

require File.expand_path('rb2html-sub.rb', File.dirname(__FILE__))

class Java2Html < Source2Html
  def initialize(formatter)
    super(formatter)

    @keywords = Hash.new
    # Keywords, The Null Literal, Boolean Literals
    %w(
      abstract    default    if            private      this
      boolean     do         implements    protected    throw
      break       double     import        public       throws
      byte        else       instanceof    return       transient
      case        extends    int           short        try
      catch       final      interface     static       void
      char        finally    long          strictfp     volatile
      class       float      native        super        while
      const       for        new           switch
      continue    goto       package       synchronized
      null true false
    ).each {|k| @keywords[k.strip] = 1 }

    @strend = '"'
  end

  def state1(ch)
    # 地の文
    case ch
    when '"'
      que_out()
      @result << @fo.begin_literal << "&quot;"
      @strend = '"'
      @state = 2
      return
    when '\''
      que_out()
      @result << @fo.begin_literal << "'"
      @strend = '\''
      @state = 2
      return
    when '/'
      if @line[0, 1] == '*'
        que_out()
        @result << @fo.begin_comment << "/*"
        @line.shift
        @state = 4 # 伝統的コメント
        return
      elsif @line[0, 1] == '/'
        que_out()
        @result << @fo.begin_comment << "/" << html_escape(@line) << @fo.end_comment
        @line = ''
        return
      end
    end
    @que << ch
  end # state1()
  
  def state2(ch)
    # "..." String Literals
    # 'a' Character Literals
    case ch
    when @strend
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

  def state4(ch)
    # コメント
    case ch
    when '*'
      if @line[0, 1] == '/'
        @result << html_escape(@que) << "*/" << @fo.end_comment
        @line.shift
        @que = ''
        @state = 1
        return
      end
    end
    @que << ch
  end

  def parse_line(s)
    @line = s
    @result << @fo.begin_comment if @state == 4 # /* ... */

    while @line != ''
      ch = @line.shift
      case @state
      when 1
        state1(ch)
      when 2
        state2(ch)
      when 4
        state4(ch)
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
    
    @result << @fo.end_comment if @state == 4
  end #parse_line()
end

if __FILE__ == $0
  f = Java2Html.new(HtmlFormatter.new)
  print f.format_io(ARGF)
end
