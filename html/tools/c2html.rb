#!/usr/local/bin/ruby

# rb2html
# Copyright (c) 2001, 2004 HORIKAWA Hisashi. All rights reserved.
#     http://www.nslabs.jp/
#     mailto:vzw00011@nifty.ne.jp

require File.expand_path('rb2html-sub.rb', File.dirname(__FILE__))
require File.expand_path('java2html.rb', File.dirname(__FILE__))

class Cpp2Html < Java2Html
  def initialize(formatter)
    super(formatter)

    @keywords = Hash.new
    %w(
      asm          do             inline             short         typeid       
      auto         double         int                signed        typename     
      bool         dynamic_cast   long               sizeof        union        
      break        else           mutable            static        unsigned     
      case         enum           namespace          static_cast   using        
      catch        explicit       new                struct        virtual      
      char         extern         operator           switch        void         
      class        false          private            template      volatile     
      const        float          protected          this          wchar_t      
      const_cast   for            public             throw         while        
      continue     friend         register           true                       
      default      goto           reinterpret_cast   try                        
      delete       if             return             typedef
      and      and_eq   bitand   bitor   compl    not 
      not_eq   or       or_eq    xor     xor_eq       
    ).each {|k| @keywords[k.strip] = 1 }

    @cpp = Hash.new
    %w(
      if ifdef ifndef elif else endif include define undef line error pragma
    ).each {|k| @cpp[k.strip] = 1 }
  end

  def parse_line(s)
    @line = s
    @result << @fo.begin_comment if @state == 4 # /* ... */

    if @state == 1
      if @line =~ /^([ \t]*)#([a-z]+)/ && @cpp[$2] 
        @result << html_escape($1) << @fo.begin_preprocessor <<
               "#" << $2 << @fo.end_preprocessor
        @line = $'
      end
    end

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
  f = Cpp2Html.new(HtmlFormatter.new)
  print f.format_io(ARGF)
end
