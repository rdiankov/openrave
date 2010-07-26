<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE xsl:stylesheet [
<!ENTITY CR "&#x0A;">
<!-- "xml:space='preserve'" is needed for use with libxslt -->
<!-- "xmlns:xsl='http://www.w3.org/1999/XSL/Transform'" is needed for
     use with xsltproc -->
<!-- Used to create a blank line -->
<!ENTITY tCR "<xsl:text xmlns:xsl='http://www.w3.org/1999/XSL/Transform' xml:space='preserve'>&CR;</xsl:text>">
<!-- Used when the line before must be ended -->
<!ENTITY tEOL "<xsl:text xmlns:xsl='http://www.w3.org/1999/XSL/Transform' xml:space='preserve'>&CR;</xsl:text>">
<!ENTITY tSP "<xsl:text xmlns:xsl='http://www.w3.org/1999/XSL/Transform' xml:space='preserve'> </xsl:text>">
]>

<!--
     Copyright (C) 2005, 2006 Stefan Merten, David Priest
     Copyright (C) 2009, 2010 Stefan Merten

     xml2rst.xsl is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published
     by the Free Software Foundation; either version 2 of the License,
     or (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
     General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
     02111-1307, USA.
-->

<!-- ********************************************************************** -->
<!-- ********************************************************************** -->

<!-- These elements in the DTD need support:

     - ``colspec`` has attribute "stub %yesorno; #IMPLIED"

     - ``document`` has attribute "title CDATA #IMPLIED"

       Probably rendered by the `.. title::` directive

     -->

<!--
Set namespace extensions. These are used as [shortname]:[tag] throughout the 
XSL-FO files.
xsl: eXtensible Stylesheet Language
u: user extensions (indicates utility 'call-template' routines defined in 
these XSL files)
data: Data elements used by the stylesheet
-->
<xsl:stylesheet
    version="1.0"
    xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
    xmlns:u="u"
    xmlns:data="a"
    exclude-result-prefixes="data"
    xmlns:str="http://exslt.org/strings"
    xmlns:dyn="http://exslt.org/dynamic"
    xmlns:math="http://exslt.org/math"
    extension-element-prefixes="str dyn math">

  <!-- xmlns:regexp="http://exslt.org/regular-expressions" not supported :-( -->

  <xsl:output
      method="text"
      omit-xml-declaration="yes"
      indent="no"/>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!-- Parameter to configure title markup; see manual page for description -->
  <xsl:param
      name="adornment"
      select="'o=o-u=u-u~u`u,u.'"/>

  <!-- Parameter for folding long lines; see manual page for description -->
  <xsl:param
      name="fold"
      select="0"/>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <xsl:variable
      name="apos"
      select='"&apos;"'/>

  <xsl:variable
      name="structural_elements"
      select="'*document*section*topic*sidebar*'"/>

  <xsl:variable
      name="structural_subelements"
      select="'*title*subtitle*docinfo*decoration*transition*'"/>

  <xsl:variable
      name="bibliographic_elements"
      select="'*address*author*authors*contact*copyright*date*field*organization*revision*status*version*'"/>

  <xsl:variable
      name="decorative_elements"
      select="'*footer*header*'"/>

  <xsl:variable
      name="simple_body_elements_no_substitution"
      select="'*comment*doctest_block*image*literal_block*paragraph*pending*raw*rubric*target*'"/>

  <xsl:variable
      name="folding_elements"
      select="concat('*comment*paragraph*rubric*attribution*caption*line*', substring-after($bibliographic_elements, '*'))"/>

  <xsl:variable
      name="simple_body_elements"
      select="concat($simple_body_elements_no_substitution, 'substitution_definition*')"/>

  <xsl:variable
      name="compound_body_elements"
      select="'*admonition*attention*block_quote*bullet_list*caution*citation*compound*danger*definition_list*enumerated_list*error*field_list*figure*footnote*hint*important*line_block*note*option_list*system_message*table*tip*warning*container*'"/>

  <xsl:variable
      name="body_elements"
      select="concat($simple_body_elements, substring-after($compound_body_elements, '*'))"/>

  <xsl:variable
      name="admonitions"
      select="'*admonition*attention*caution*danger*error*hint*important*note*tip*warning*'"/>

  <xsl:variable
      name="simple_body_subelements"
      select="'*attribution*caption*classifier*colspec*field_name*label*line*option_argument*option_string*term*'"/>

  <xsl:variable
      name="compound_body_subelements"
      select="'*definition*definition_list_item*description*entry*field*field_body*legend*list_item*option*option_group*option_list_item*row*tbody*tgroup*thead*'"/>

  <xsl:variable
      name="inline_elements"
      select="'*abbreviation*acronym*citation_reference*emphasis*footnote_reference*generated*image*inline*literal*problematic*reference*strong*subscript*substitution_reference*superscript*target*title_reference*raw*'"/>

  <xsl:variable
      name="inline_containers"
      select="concat($simple_body_elements_no_substitution, substring-after($inline_elements, '*'))"/>

  <xsl:variable
      name="directives"
      select="'*admonition*attention*caution*comment*danger*error*footnote*hint*important*note*tip*warning*image*figure*topic*sidebar*rubric*meta*raw*citation*compound*substitution_definition*container*'"/>

  <xsl:variable
      name="titled_elements"
      select="'*sidebar*topic*admonition*'"/>

  <xsl:variable
      name="blank_after"
      select="concat($structural_elements, substring-after($structural_subelements, '*'), substring-after($body_elements, '*'))"/>

  <xsl:variable
      name="adornment_characters"
      select="concat($apos, '!&quot;#$%&amp;()*+,-./:;&lt;=&gt;?@[\]^_`{|}~')"/>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!--
  Content Model: ((title, subtitle?)?, docinfo?, decoration?,
  %structure.model;)

  Attributes:    The document element contains only the common attributes: ids,
  names, dupnames, source, and classes.

  Depending on the source of the data and the stage of processing, the 
  "document" may not initially contain a "title". A document title is not 
  directly representable in reStructuredText. Instead, a lone top-level section
  may have its title promoted to become the document title, and similarly for a
  lone second-level (sub)section's title to become the document subtitle. The 
  "docinfo" may be transformed from an initial field_list, and "decoration" is 
  usually constructed programmatically.
  -->
  <!-- == structural_element -->
  <xsl:template
      match="document">
    <xsl:if
	test="//generated[@classes = 'sectnum']">
      <xsl:text>.. section-numbering::</xsl:text>
      &tEOL;
      &tCR;
    </xsl:if>
    <xsl:call-template
	name="u:outputClass">
      <xsl:with-param
	  name="blankAfter"
	  select="true()"/>
    </xsl:call-template>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (title, %structure.model;)
  Attributes:    The section element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == structural_element -->
  <xsl:template
      match="section">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:blank"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- == structural_element -->
  <xsl:template
      match="section[@classes = 'system-messages']"/>
  <!-- Ignore system messages completely -->
  <!-- This should be really in `generated' -->

  <!-- ******************************************************************** -->

  <!--
  Content Model: (title, subtitle?, (%body.elements;)+)
  Attributes:    The sidebar element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == structural_element == directive -->
  <xsl:template
      match="sidebar">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. sidebar:: </xsl:text>
    <xsl:value-of
	select="title"/>
    &tEOL;
    <xsl:if
	test="subtitle">
      <xsl:call-template
	  name="u:param">
	<xsl:with-param
	    name="name"
	    select="'subtitle'"/>
	<xsl:with-param
	    name="value"
	    select="subtitle"/>
	<xsl:with-param
	    name="ancestors"
	    select="ancestor-or-self::*"/>
      </xsl:call-template>
    </xsl:if>
    <xsl:call-template
	name="u:params"/>
    <!-- Always blank line after parameter block -->
    &tCR;
    <xsl:apply-templates
	select="*[not(self::title) and not(self::subtitle)]"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (title?, (%body.elements;)+)
  Attributes:    The topic element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == structural_element == directive -->
  <xsl:template
      match="topic">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. topic:: </xsl:text>
    <xsl:value-of
	select="title"/>
    &tEOL;
    <xsl:call-template
	name="u:params"/>
    <xsl:apply-templates
	select="*[not(self::title)]"/>
  </xsl:template>

  <!-- == structural_element == directive -->
  <xsl:template
      match="topic[starts-with(@classes, 'contents')]">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. contents:: </xsl:text>
    <xsl:apply-templates
	select="title"/>
    &tEOL;
    <xsl:call-template
	name="u:params">
      <xsl:with-param
	  name="params"
	  select="@*[name() != 'ids' and name() != 'names' and name() != 'classes']"/>
    </xsl:call-template>
    <xsl:variable
	name="isLocal"
	select="substring-before(@classes, ' local')"/>
    <xsl:variable
	name="realClassesLocal"
	select="normalize-space(substring-after(@classes, 'contents'))"/>
    <xsl:variable
	name="realClassesNode">
      <xsl:choose>
	<xsl:when
	    test="$isLocal">
	  <xsl:value-of
	      select="normalize-space(substring-before($realClassesLocal, 'local'))"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="$realClassesLocal"/>
	</xsl:otherwise>
      </xsl:choose>
    </xsl:variable>
    <xsl:variable
	name="realClasses"
	select="string($realClassesNode)"/>
    <xsl:if
	test="$isLocal">
      <xsl:call-template
	  name="u:param">
	<xsl:with-param
	    name="name"
	    select="'local'"/>
	<xsl:with-param
	    name="value"
	    select="''"/>
	<xsl:with-param
	    name="ancestors"
	    select="ancestor-or-self::*"/>
      </xsl:call-template>
    </xsl:if>
    <xsl:if
	test="$realClasses">
      <xsl:call-template
	  name="u:param">
	<xsl:with-param
	    name="name"
	    select="'class'"/>
	<xsl:with-param
	    name="value"
	    select="$realClasses"/>
        <xsl:with-param
	    name="ancestors"
	    select="ancestor-or-self::*"/>
      </xsl:call-template>
    </xsl:if>
    <!-- Autogenerated content is discarded -->
    &tCR;
  </xsl:template>

  <!-- == structural_element == directive -->
  <xsl:template
      match="topic[@classes='dedication' or @classes='abstract']">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>:</xsl:text>
    <xsl:apply-templates
	select="title"/>
    <xsl:text>: </xsl:text>
    &tEOL;
    <xsl:apply-templates
	select="*[not(self::title)]"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (title, (%body.elements;)+)
  Attributes:    The admonition element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element == directive -->
  <xsl:template
      match="admonition">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. admonition:: </xsl:text>
    <xsl:apply-templates
	select="title"/>
    &tEOL;
    <xsl:call-template
	name="u:params">
      <xsl:with-param
	  name="params"
	  select="@*[name() != 'classes' or not(starts-with(., 'admonition-'))]"/>
    </xsl:call-template>
    <xsl:call-template
	name="u:indent"/>
    <xsl:apply-templates
	select="*[not(self::title)]"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)+
  Attributes:    The note element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element == directive -->
  <xsl:template
      match="attention | caution | danger | error | hint | important | note | tip | warning">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. </xsl:text>
    <xsl:value-of
	select="name()"/>
    <xsl:text>:: </xsl:text>
    <xsl:call-template
	name="u:params">
      <xsl:with-param
	  name="params"
	  select="@*[name() != 'classes']"/>
    </xsl:call-template>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (header?, footer?)
  Attributes:    The decoration element contains only the common attributes:
  ids, names, dupnames, source, and classes.

  Although the content model doesn't specifically require contents, no empty 
  decoration elements are ever created.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="//document/decoration">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- TODO To be rendered as `.. header::` directive -->
  <!-- == decorative_element -->
  <xsl:template
      match="//document/decoration/header">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- TODO To be rendered as `.. footer::` directive -->
  <!-- == decorative_element -->
  <xsl:template
      match="//document/decoration/footer">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%bibliographic.elements;)+
  Attributes:    The docinfo element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="docinfo">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:blank"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: ((author, organization?, address?, contact?)+)
  Attributes:    The authors element contains only the common attributes: ids,
  names, dupnames, source, and classes.

  In reStructuredText, multiple author's names are separated with semicolons 
  (";") or commas (","); semicolons take precedence. There is currently no way 
  to represent the author's organization, address, or contact in a 
  reStructuredText "Authors" field.
  -->
  <!-- == bibliographic_element == folding_element -->
  <xsl:template
      match="docinfo/authors">
    <xsl:call-template
	name="u:outputFolding">
      <xsl:with-param
	  name="prefix">
	<xsl:text>:</xsl:text>
	<xsl:value-of
	    select="name()"/>
	<xsl:text>: </xsl:text>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <!--
  Content Model: %text.model;
  Attributes:    All docinfo elements contains the common attributes (ids,
  names, dupnames, source, and classes)
	      Some docinfo elements also have xml:space.
  -->
  <xsl:template
      match="docinfo/authors/*">
    <xsl:apply-templates/>
    <!-- no semicolon after final author -->
    <xsl:if
	test="generate-id(current()) != generate-id(../*[last()])">
      <xsl:text>; </xsl:text>
    </xsl:if>
  </xsl:template>

  <!--
  Content Model: (field_name, field_body)
  Attributes:    The field element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == bibliographic_element -->
  <xsl:template
      match="docinfo/field">
    <!-- contents handled by ordinary field lists -->
    <xsl:apply-templates/>
    <!-- Supply an EOL because following elements do not recognize this -->
    &tEOL;
  </xsl:template>

  <!--
  Content Model: %text.model;
  Attributes:    All docinfo elements contains the common attributes (ids,
  names, dupnames, source, and classes)
	      Some docinfo elements also have xml:space.
  -->
  <!-- == bibliographic_element == folding_element -->
  <xsl:template
      match="docinfo/*[name()!='authors' and name()!='field']">
    <xsl:call-template
	name="u:outputFolding">
      <xsl:with-param
	  name="prefix">
	<xsl:text>:</xsl:text>
	<xsl:value-of
	    select="name()"/>
	<xsl:text>: </xsl:text>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: EMPTY
  Attributes:    The transition element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="transition">
    <xsl:call-template
	name="u:outputClass"/>
    &tCR; <!-- req: blank line before -->
    <xsl:text>-----</xsl:text>
    &tEOL;
    <!-- Add a required blank line after unless class follows immediately -->
    <xsl:if
	test="not(following-sibling::*[1]/@classes)">
      &tCR;
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!--
  IFF there is a /document/title element, it is the publication's title.  All 
  other titles will appear within sections.

  Content Model: %text.model;
  Attributes:    The title element contains the common attributes (ids, names, 
  dupnames, source, and classes), plus refid and auto.
      refid is used as a backlink to a table of contents entry.
      auto is used to indicate (with value "1") that the title has been
  numbered automatically.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="//document/title">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:variable
	name="textWS">
      <!-- Catch the title text as it is rendered so its length can be
           determined -->    
      <xsl:apply-templates/>
    </xsl:variable>
    <xsl:variable
	name="text"
	select="normalize-space($textWS)"/>
    <xsl:variable
	name="length"
	select="string-length($text)"/>
    <xsl:call-template
	name="u:overline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="1"/>
    </xsl:call-template>
    <xsl:value-of
	select="$text"/>
    &tEOL;    
    <xsl:call-template
	name="u:underline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="1"/>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Title Underlines are defined by their position within the tree.

  Content Model: %text.model;
  Attributes:    The title element contains the common attributes (ids, names, 
  dupnames, source, and classes), plus refid and auto.
      refid is used as a backlink to a table of contents entry.
      auto is used to indicate (with value "1") that the title has been
  numbered automatically.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="section/title">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:variable
	name="textWS">
      <!-- catch the title text as it is rendered -->    
      <xsl:apply-templates/>
    </xsl:variable>
    <xsl:variable
	name="text"
	select="normalize-space($textWS)"/>
    <xsl:variable
	name="length"
	select="string-length($text)"/>
    <xsl:variable
	name="depth"
	select="count(ancestor::section)"/>
    <xsl:call-template
	name="u:overline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="$depth + 2"/>
    </xsl:call-template>
    <xsl:value-of
	select="$text"/>
    &tEOL;    
    <xsl:call-template
	name="u:underline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="$depth + 2"/>
    </xsl:call-template>
    <!-- Add a blank line after unless structure follows immediately -->
    <xsl:if
	test="not(contains(concat($structural_elements, $compound_body_elements), concat('*', name(following-sibling::*[1]), '*')) or following-sibling::*[1]/@classes)">
      &tCR;
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The title element contains the common attributes (ids, names, 
  dupnames, source, and classes), plus refid and auto.
      refid is used as a backlink to a table of contents entry.
      auto is used to indicate (with value "1") that the title has been
  numbered automatically.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="title">
    <xsl:call-template
	name="u:outputClass">
      <xsl:with-param
	  name="alreadyBlanked"
	  select="true()"/>
    </xsl:call-template>
    <!-- blank line provided by parent -->
    <xsl:apply-templates/>
    <!-- no EOL: provided by parent -->
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  IFF there is a /document/title element, it is the publication's title.  All 
  other titles will appear within sections.

  Content Model: %text.model;
  Attributes:    The subtitle element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="//document/subtitle">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:variable
	name="textWS">
      <!-- Catch the title text as it is rendered -->    
      <xsl:apply-templates/>
    </xsl:variable>
    <xsl:variable
	name="text"
	select="normalize-space($textWS)"/>
    <xsl:variable
	name="length"
	select="string-length($text)"/>

    <!-- always a blank line above -->
    &tCR;
    <xsl:call-template
	name="u:overline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="2"/>
    </xsl:call-template>
    <xsl:value-of
	select="$text"/>
    &tEOL;    
    <xsl:call-template
	name="u:underline">
      <xsl:with-param
	  name="length"
	  select="$length"/>
      <xsl:with-param
	  name="depth"
	  select="2"/>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The subtitle element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == structural_subelement -->
  <xsl:template
      match="sidebar/subtitle">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:indent"/>
    <xsl:apply-templates/>
    &tEOL;
    &tCR;
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The comment element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == simple_body_element == folding_element == directive -->
  <xsl:template
      match="comment">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:call-template
	name="u:outputFolding">
      <xsl:with-param
	  name="prefix">
	<xsl:text>.. </xsl:text>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The doctest_block element contains the common attributes (ids,
  names, dupnames, source, and classes), plus xml:space.
  -->
  <!-- == simple_body_element -->
  <xsl:template
      match="doctest_block">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:apply-templates/>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- An image element can have various roles; they are all covered here -->
  <!-- == simple_body_element == inline_element == directive -->
  <xsl:template
      match="image">
    <xsl:choose>
      <xsl:when
	  test="contains($inline_containers, concat('*', name(..), '*')) and @alt">
	<!-- An inline image with an `@alt' - must be a substitution
             reference -->
	<xsl:text>|</xsl:text>
	<!-- Original text is lost - use what we have -->
	<xsl:value-of
	    select="@alt"/>
	<xsl:text>|</xsl:text>
      </xsl:when>
      <xsl:otherwise>
	<!-- A directive -->
	<xsl:if
	    test="not(parent::figure)">
	  <xsl:if
	      test="not(parent::substitution_definition)">
	    <xsl:call-template
		name="u:BandI"/>
	    <xsl:text>.. </xsl:text>
	  </xsl:if>
	  <xsl:text>image:: </xsl:text>
	</xsl:if>
	<xsl:value-of
	    select="@uri"/>
	&tEOL;
	<xsl:choose>
	  <xsl:when
	      test="parent::figure">
	    <!-- `@classes' is special because it is in the parent -->
	    <xsl:if
		test="../@classes">
	      <xsl:call-template
		  name="u:param">
		<xsl:with-param
		    name="name"
		    select="'figclass'"/>
		<xsl:with-param
		    name="value"
		    select="../@classes"/>
		<xsl:with-param
		    name="ancestors"
		    select="ancestor::*"/>
	      </xsl:call-template>
	    </xsl:if>
	    <!-- `@align' is special because it is in the parent -->
	    <xsl:if
		test="../@align">
	      <xsl:call-template
		  name="u:param">
		<xsl:with-param
		    name="name"
		    select="'align'"/>
		<xsl:with-param
		    name="value"
		    select="../@align"/>
		<xsl:with-param
		    name="ancestors"
		    select="ancestor::*"/>
	      </xsl:call-template>
	    </xsl:if>
	    <xsl:call-template
		name="u:params">
	      <!-- `figure' would add one level of indentation --> 
	      <xsl:with-param
		  name="ancestors"
		  select="ancestor::*"/>
	    </xsl:call-template>
	  </xsl:when>
	  <xsl:when
	      test="parent::substitution_definition">
	    <xsl:call-template
		name="u:params">
	      <!-- `@alt' only for the real images --> 
	      <xsl:with-param
		  name="params"
		  select="@*[name() != 'alt']"/>
	    </xsl:call-template>
	  </xsl:when>
	  <xsl:otherwise>
	    <xsl:call-template
		name="u:params">
	      <xsl:with-param
		  name="params"
		  select="@*[name() != 'ids' and name() != 'names']"/>
	    </xsl:call-template>
	  </xsl:otherwise>
	</xsl:choose>
	<xsl:if
	    test="parent::reference">
	  <!-- A clickable image -->
	  <xsl:call-template
	      name="u:param">
	    <xsl:with-param
		name="name"
		select="'target'"/>
	    <xsl:with-param
		name="value">
	      <xsl:choose>
		<xsl:when
		    test="../@name">
		  <!-- An internal link -->
		  <xsl:call-template
		      name="u:inlineReference">
		    <xsl:with-param
			name="text"
			select="../@refid"/>
		  </xsl:call-template>
		</xsl:when>
		<xsl:otherwise>
		  <!-- An external link -->
		  <xsl:value-of
		      select="../@refuri"/>
		</xsl:otherwise>
	      </xsl:choose>
	    </xsl:with-param>
	    <xsl:with-param
		name="ancestors"
		select="ancestor-or-self::*"/>
	  </xsl:call-template>
	</xsl:if>
	<!-- Always blank line after parameter block -->
	&tCR;
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (line_block | line)+
  Attributes:    The line_block element contains the common attributes (ids, 
  names, dupnames, source, and classes), plus xml:space.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="line_block">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:variable
	name="isEmbedded"
	select="name(..) = 'line_block'"/>
    <xsl:if
	test="not($isEmbedded)">
      <xsl:call-template
	  name="u:blank"/>
    </xsl:if>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The line element contains the common attributes (ids, 
  names, dupnames, source, and classes).
  -->
  <!-- == simple_body_subelement == folding_element -->
  <xsl:template
      match="line">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:variable
	name="indent">
      <xsl:call-template
	  name="u:indent"/>
    </xsl:variable>
    <xsl:value-of
	select="$indent"/>
    <xsl:choose>
      <xsl:when
	  test="node()">
	<!-- Only for non-empty lines -->
	<xsl:variable
	    name="lineBlockIndent">
	  <!-- Very special indendation for nested `line_block's -->
	  <xsl:for-each
	      select="ancestor::line_block[position() > 1]">
	    <xsl:value-of
		select="str:padding(4)"/>
	  </xsl:for-each>
	</xsl:variable>
	<xsl:call-template
	    name="u:outputFolding">
	  <xsl:with-param
	      name="prefix">
	    <xsl:text>| </xsl:text>
	    <xsl:value-of
		select="$lineBlockIndent"/>
	  </xsl:with-param>
	  <xsl:with-param
	      name="indent"
	      select="concat($indent, '  ', $lineBlockIndent)"/>
	</xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
	<xsl:text>|</xsl:text>
	&tEOL;
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The literal_block element contains the common attributes (ids,
  names, dupnames, source, and classes), plus xml:space.
  -->
  <!-- == simple_body_element == directive -->
  <xsl:template
      match="literal_block">
    <xsl:choose>
      <xsl:when
	  test=".//*[contains($inline_elements, concat('*', name(), '*'))]">
	<!-- If it contains inline elements this is a parsed-literal -->
	<xsl:call-template
	    name="u:BandI"/>
	<xsl:text>.. parsed-literal::</xsl:text>
	&tEOL;
	<xsl:call-template
	    name="u:params"/>
	&tCR;
      </xsl:when>
      <xsl:otherwise>
	<xsl:call-template
	    name="u:outputClass"/>
	<!-- TODO Support for the (fully) minimized style would be nice but
	          is difficult to accomplish because there is a linefeed
	          already when we arrive here :-( -->
	<xsl:call-template
	    name="u:BandI"/>
	<xsl:text>::</xsl:text>
	&tEOL;
	&tCR;
      </xsl:otherwise>
    </xsl:choose>
    <xsl:variable
	name="isQuotedLiteral">
      <xsl:call-template
	  name="u:isQuotedLiteral"/>
    </xsl:variable>
    <!-- Indent correctly depending on quoted literal or not -->
    <xsl:choose>
      <xsl:when
	  test="string-length($isQuotedLiteral)">
	<xsl:call-template
	    name="u:indent">
	  <xsl:with-param
	      name="ancestors"
	      select="ancestor::*"/>
	</xsl:call-template>
	<xsl:apply-templates
	    mode="quotedLiteral"/>
      </xsl:when>
      <xsl:otherwise>
	<xsl:call-template
	    name="u:indent">
	  <xsl:with-param
	      name="ancestors"
	      select="ancestor-or-self::*"/>
	</xsl:call-template>
	<xsl:apply-templates/>
      </xsl:otherwise>
    </xsl:choose>
    &tEOL;
  </xsl:template>

  <!-- Indent a text of a quoted literal containing line feeds correctly -->
  <xsl:template
      match="text()"
      mode="quotedLiteral">
    <xsl:call-template
	name="u:indentLF">
      <xsl:with-param
	  name="indent">
	<xsl:call-template
	    name="u:indent">
	  <xsl:with-param
	      name="ancestors"
	      select="ancestor::*[position() > 1]"/>
	</xsl:call-template>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <!-- Determine whether `$text' is a quoted literal and return the quote
       character if so -->
  <xsl:template
      name="u:isQuotedLiteral">
    <xsl:param
	name="text"
	select="text()"/>
    <xsl:param
	name="quote"
	select="substring($text, 1, 1)"/>
    <xsl:if
	test="contains($adornment_characters, $quote) and substring($text, 1, 1) = $quote">
      <!-- Given quote is an adornment character and first character is this
           quote -->
      <xsl:choose>
	<xsl:when
	    test="contains($text, '&#xA;')">
	  <!-- Test the remaining lines -->
	  <xsl:call-template
	      name="u:isQuotedLiteral">
	    <xsl:with-param
		name="text"
		select="substring-after($text, '&#xA;')"/>
	    <xsl:with-param
		name="quote"
		select="$quote"/>
	  </xsl:call-template>
	</xsl:when>
	<xsl:otherwise>
	  <!-- No more lines to test so this is a quoted literal -->
	  <xsl:value-of
	      select="$quote"/>
	</xsl:otherwise>
      </xsl:choose>
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The paragraph element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == simple_body_element == folding_element -->
  <xsl:template
      match="paragraph">
    <xsl:variable
	name="previous"
	select="preceding-sibling::*[1]"/>
    <!-- Do indent except first element in some compound elements -->
    <xsl:variable
	name="needsIndent"
	select="($previous or not(parent::list_item or parent::field_body or contains($admonitions, concat('*', name(..), '*')))) and name($previous) != 'label'"/>
    <!-- Blank line in front if following a body element, except first
         elements in some compound elements -->
    <xsl:variable
	name="needsBlank"
	select="($previous or not(parent::list_item or ../parent::option_list_item or parent::field_body or parent::document or contains($admonitions, concat('*', name(..), '*')))) and (not($previous) or contains($body_elements, concat('*', name($previous), '*')) or name($previous) = 'title' and contains($titled_elements, concat('*', name(..), '*')) or name($previous) = 'docinfo')"/>
    <xsl:if
	test="$needsBlank">
      &tCR;
    </xsl:if>
    <xsl:if
	test="$needsIndent">
      <xsl:call-template
	  name="u:indent"/>
    </xsl:if>
    <xsl:if
	test="@classes">
      <!-- This paragraph has a classes attribute - always needs newline and
           indent -->
      <xsl:call-template
	  name="u:outputClass">
	<xsl:with-param
	    name="alreadyBlanked"
	    select="$needsBlank"/>
	<xsl:with-param
	    name="alreadyIndented"
	    select="$needsIndent"/>
      </xsl:call-template>
      <xsl:call-template
	  name="u:BandI"/>
    </xsl:if>
    <xsl:call-template
	name="u:outputFolding"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_element -->
  <xsl:template
      match="pending">
    <xsl:call-template
	name="u:notSupported"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_element == inline_element == directive -->
  <xsl:template
      match="raw">
    <xsl:choose>
      <xsl:when
	  test="contains($inline_containers, concat('*', name(..), '*'))">
	<!-- Used as a custom role -->
	<!-- TODO `role' directives must be generated for user-defined raw
	          roles. -->
	<!-- The name of the custom role is not contained in the input -->
	<xsl:text>:RAW-ROLE:`</xsl:text>
	<xsl:apply-templates/>
	<xsl:text>`</xsl:text>
      </xsl:when>
      <xsl:otherwise>
	<!-- A directive -->
	<xsl:call-template
	    name="u:outputClass"/>
	<xsl:call-template
	    name="u:BandI"/>
	<xsl:text>.. raw:: </xsl:text>
	<xsl:value-of
	    select="@format"/>
	&tEOL;
	<xsl:call-template
	    name="u:params">
	  <xsl:with-param
	      name="params"
	      select="@*[name() != 'format' and name() != 'classes']"/>
	</xsl:call-template>
	&tCR;
	<xsl:call-template
	    name="u:indent">
	  <xsl:with-param
	      name="ancestors"
	      select="ancestor-or-self::*"/>
	</xsl:call-template>
	<xsl:apply-templates/>
	&tEOL;
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_element == folding_element == directive -->
  <xsl:template
      match="rubric">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:call-template
	name="u:outputFolding">
      <xsl:with-param
	  name="prefix">
	<xsl:text>.. rubric:: </xsl:text>
      </xsl:with-param>
    </xsl:call-template>
    <xsl:call-template
	name="u:params"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == compound_body_element == directive -->
  <xsl:template
      match="compound">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. compound::</xsl:text>
    &tEOL;
    <xsl:call-template
	name="u:params"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == compound_body_element == directive -->
  <xsl:template
      match="container">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. container::</xsl:text>
    <xsl:if
	test="@classes">
      &tSP;
      <xsl:value-of
	  select="@classes"/>
    </xsl:if>
    &tEOL;
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_element == directive -->
  <xsl:template
      match="substitution_definition">
    <!-- More than one child or not a directive is a replacement text -->
    <xsl:variable
	name="isReplace"
	select="not(* and count(node()) = 1 and contains($directives, concat('*', name(*[1]), '*')))"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:variable
	name="prefix">
      <xsl:text>.. |</xsl:text>
      <xsl:call-template
	  name="u:outputNames"/>
      <xsl:text>| </xsl:text>
    </xsl:variable>
    <xsl:choose>
      <xsl:when
	  test="$isReplace">
      <!-- TODO Substitution references for replace can not be found because
	        they are not marked as such; embedding them in `generated'
                would be nice -->
	<xsl:call-template
	    name="u:outputFolding">
	  <xsl:with-param
	      name="prefix">
	    <xsl:value-of
		select="$prefix"/>
	    <xsl:text>replace:: </xsl:text>
	  </xsl:with-param>
	</xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
	<xsl:value-of
	    select="$prefix"/>
	<xsl:apply-templates/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: ((%body.elements;)+, attribution?)
  Attributes:    The block_quote element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="block_quote">
    <xsl:if
	test="@classes = 'epigraph' or @classes = 'highlights' or @classes = 'pull-quote'">
      <xsl:call-template
	  name="u:BandI"/>
      <xsl:text>.. </xsl:text>
      <xsl:value-of
	  select="@classes"/>
      <xsl:text>::</xsl:text>
      &tEOL;
      <xsl:call-template
	  name="u:params"/>
    </xsl:if>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_subelement == folding_element -->
  <xsl:template
      match="attribution">
    <xsl:call-template
	name="u:outputClass"/>
    <!-- blank line between quote and attribution -->
    &tCR;
    <xsl:call-template
	name="u:indent"/>
    <xsl:call-template
	name="u:outputFolding">
      <xsl:with-param
	  name="prefix">
	<xsl:text>-- </xsl:text>
      </xsl:with-param>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == compound_body_element == directive -->
  <xsl:template
      match="citation">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. [</xsl:text>
    <xsl:value-of
	select="label"/>
    <xsl:text>] </xsl:text>
    <xsl:apply-templates
	select="*[not(self::label)]"/>
  </xsl:template>

  <!-- == simple_body_subelement -->
  <xsl:template
      match="citation/label">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == compound_body_element == directive -->
  <xsl:template
      match="figure">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. figure:: </xsl:text>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == simple_body_subelement == folding_element -->
  <xsl:template
      match="caption">
    <xsl:call-template
	name="u:indent"/>
    <xsl:call-template
	name="u:outputFolding"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == compound_body_subelement -->
  <xsl:template
      match="legend">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- TODO Footnotes should continue on line of definition -->

  <!-- user-numbered footnotes lack @auto -->
  <!-- == compound_body_element == directive -->
  <xsl:template
      match="footnote[not(@auto)]">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. [</xsl:text>
    <xsl:apply-templates
	select="label"/>
    <xsl:text>] </xsl:text>
    <xsl:apply-templates
	select="*[not(self::label)]"/>
  </xsl:template>

  <!-- autonumbered footnotes have @auto -->
  <!-- if the target footnote_reference@names matches its label, it was not a
       numbered-name footnote -->
  <!-- == compound_body_element == directive -->
  <xsl:template
      match="footnote[@auto='1']">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. [#</xsl:text>
    <xsl:if
	test="@names = @ids">
      <xsl:call-template
	  name="u:outputNames"/>
    </xsl:if>
    <xsl:text>] </xsl:text>
    <xsl:apply-templates
	select="*[not(self::label)]"/>
  </xsl:template>

  <!-- autosymboled footnotes have @auto -->
  <!-- == compound_body_element == directive -->
  <xsl:template
      match="footnote[@auto='*']">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>.. [*] </xsl:text>
    <xsl:apply-templates
	select="*[not(self::label)]"/>
  </xsl:template>

  <!-- == compound_body_element == directive -->
  <xsl:template
      match="footnote[starts-with(@names, 'TARGET_NOTE:\ ')]">
    <!-- This is not a footnote but a hint for a directive -->
    <xsl:if
	test="generate-id(//footnote[starts-with(@names, 'TARGET_NOTE:\ ')][1]) = generate-id(.)">
      <!-- Only for the first one -->
      <xsl:call-template
	  name="u:BandI"/>
      <!-- TODO May have a `classes` attribute -->
      <xsl:text>.. target-notes::</xsl:text>
      &tEOL;
    </xsl:if>
  </xsl:template>

  <!-- == simple_body_subelement -->
  <xsl:template
      match="footnote/label">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (list_item +)
  Attributes:    The bullet_list element contains the common attributes (ids,
  names, dupnames, source, and classes), plus bullet.
      bullet is used to record the style of bullet from the input data.
      In documents processed from reStructuredText, it contains one of "-",
  "+", or "*". It may be ignored in processing.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="bullet_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates
	mode="bullet_list"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (definition_list_item +)
  Attributes:    The definition_list element contains only the common
  attributes: ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="definition_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (term, classifier?, definition)
  Attributes:    The definition_list_item element contains only the common 
  attributes: ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="definition_list_item">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The term element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == simple_body_subelement -->
  <xsl:template
      match="term">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The classifier element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == simple_body_subelement -->
  <xsl:template
      match="classifier">
    <xsl:text> : </xsl:text>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)+
  Attributes:    The definition element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="definition">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (list_item +)
  Attributes:    The enumerated_list element contains the common attributes
  (ids, names, dupnames, source, and classes), plus enumtype, prefix, suffix, and
  start.
      enumtype is used to record the intended enumeration sequence, one
  of "arabic" (1, 2, 3, ...), "loweralpha" (a, b, c, ..., z), "upperalpha" (A,
  B, C, ..., Z), "lowerroman" (i, ii, iii, iv, ..., mmmmcmxcix [4999]), or 
  "upperroman" (I, II, III, IV, ..., MMMMCMXCIX [4999]).
      prefix stores the formatting characters used before the enumerator. In 
  documents originating from reStructuredText data, it will contain either "" 
  (empty string) or "(" (left parenthesis). It may or may not affect
  processing.
      suffix stores the formatting characters used after the enumerator. In 
  documents originating from reStructuredText data, it will contain either "." 
  (period) or ")" (right parenthesis). Depending on the capabilities of the 
  output format, this attribute may or may not affect processing.
      start contains the ordinal value of the first item in the list, in 
  decimal. For lists beginning at value 1 ("1", "a", "A", "i", or "I"), this 
  attribute may be omitted.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="enumerated_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates
	mode="enumerated_list"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (field +)
  Attributes:    The field_list element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="field_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (field_name, field_body)
  Attributes:    The field element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="field">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ********************************************************************** -->

  <!--
  Content Model: %text.model;
  Attributes:    The field_name element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == simple_body_subelement -->
  <xsl:template
      match="field_name">
    <xsl:text>:</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>: </xsl:text>
    <!-- no EOL: field_body starts on same line -->
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)*
  Attributes:    The field_body element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="field_body">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (option_list_item +)
  Attributes:    The option_list element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="option_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:blank"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (option_group, description)
  Attributes:    The option_list_item element contains only the common
  attributes: ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="option_list_item">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:indent"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (option_group, description)
  Attributes:    The option_group element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="option_group">
    <xsl:apply-templates/>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (option_string, option_argument *)
  Attributes:    The option element contains only the common attributes: ids,
  names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="option">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
    <xsl:if
	test="generate-id(current()) != generate-id(../*[last()])">
      <!-- No comma after final option -->
      <xsl:text>, </xsl:text>
    </xsl:if>
    <!-- no EOL: description on same line -->
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (#PCDATA)
  Attributes:    The option_string element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == simple_body_subelement -->
  <xsl:template
      match="option_string">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (#PCDATA)
  Attributes:    The option_argument element contains the common attributes
  (ids,  names, dupnames, source, and classes), plus delimiter.
      delimiter contains the text preceding the option_argument:
  either the text separating it from the option_string (typically
  either "=" or " ")
  or the text between option arguments (typically either "," or " ").
  -->
  <!-- == simple_body_subelement -->
  <xsl:template
      match="option_argument">
    <xsl:value-of
	select="@delimiter"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)+
  Attributes:    The description element contains only the common attributes:
  ids, names, dupnames, source, and classes.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="description">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:apply-templates/>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)+
  Attributes:    The list_item element contains only the common attributes:
  ids, names, dupnames, source, and classes

  BULLET LIST
      bullet is used to record the style of bullet from the input data.
      In documents processed from reStructuredText, it contains one of "-",
  "+", or "*". It may be ignored in processing.
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="list_item"
      mode="bullet_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:value-of
	select="../@bullet"/>    
    &tSP;  <!-- space after bullet -->
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  ENUMERATED LIST
      enumtype is used to record the intended enumeration sequence, one of 
  "arabic" (1, 2, 3, ...), "loweralpha" (a, b, c, ..., z), "upperalpha" (A, B, 
  C, ..., Z), "lowerroman" (i, ii, iii, iv, ..., mmmmcmxcix [4999]), or 
  "upperroman" (I, II, III, IV, ..., MMMMCMXCIX [4999]).
      prefix stores the formatting characters used before the enumerator. In 
  documents originating from reStructuredText data, it will contain either "" 
  (empty string) or "(" (left parenthesis). It may or may not affect
  processing.
      suffix stores the formatting characters used after the enumerator. In 
  documents originating from reStructuredText data, it will contain either "." 
  (period) or ")" (right parenthesis). Depending on the capabilities of the 
  output format, this attribute may or may not affect processing.
      start contains the ordinal value of the first item in the list, in 
  decimal. For lists beginning at value 1 ("1", "a", "A", "i", or "I"), this 
  attribute may be omitted.

  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="list_item"
      mode="enumerated_list">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:BandI"/>
    <xsl:call-template
	name="u:outputEnumerator"/>
    <xsl:apply-templates/>
  </xsl:template>

  <!-- Outputs a complete enumerator when called in an
       enumerated_list/list_item -->
  <xsl:template
      name="u:outputEnumerator">
    <!-- Use parent's numeration attribute -->
    <xsl:variable
	name="enumType"
	select="../@enumtype"/>
    <!-- Determine starting point -->
    <xsl:variable
	name="start">
      <xsl:choose>
	<xsl:when
	    test="../@start">
	  <xsl:value-of
	      select="../@start"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="1"/>
	</xsl:otherwise>
      </xsl:choose>
    </xsl:variable>
    <!-- Determine position of this item in its real context -->
    <xsl:variable
	name="position">
      <xsl:variable
	  name="wanted"
	  select="generate-id()"/>
      <!-- Generate the right current node list -->
      <xsl:for-each
	  select="../list_item">
	<xsl:if
	    test="generate-id() = $wanted">
	  <xsl:value-of
	      select="position()"/>
	</xsl:if>
      </xsl:for-each>
    </xsl:variable>
    <!-- Determine encoding of the number for the given numeration -->
    <xsl:variable
	name="cur">
      <xsl:call-template
	  name="u:position2Enumerator">
	<xsl:with-param
	    name="enumType"
	    select="$enumType"/>
	<xsl:with-param
	    name="position"
	    select="$position"/>
	<xsl:with-param
	    name="start"
	    select="$start"/>
      </xsl:call-template>
    </xsl:variable>
    <!-- Determine encoding of the maximum number -->
    <xsl:variable
	name="max">
      <xsl:call-template
	  name="u:position2Enumerator">
	<xsl:with-param
	    name="enumType"
	    select="$enumType"/>
	<xsl:with-param
	    name="position"
	    select="count(../list_item)"/>
	<xsl:with-param
	    name="start"
	    select="$start"/>
      </xsl:call-template>
    </xsl:variable>
    <!-- Output complete enumerator -->
    <xsl:value-of
	select="../@prefix"/>
    <xsl:value-of
	select="$cur"/>
    <xsl:value-of
	select="../@suffix"/>
    <!-- Output at least one trailing space -->
    &tSP;
    <!-- Output more whitespace to align with the maximum enumerator -->
    <xsl:if
	test="$enumType != 'lowerroman' and $enumType != 'upperroman'">
      <!-- Assumes that the maximum number has maximum string length -->
      <xsl:value-of
	  select="str:padding(string-length($max) - string-length($cur))"/>
    </xsl:if>
  </xsl:template>

  <!-- Determine the right ordinal enumerator based on the parameters -->
  <xsl:template
      name="u:position2Enumerator">
    <xsl:param
	name="enumType"/>
    <xsl:param
	name="start"/>
    <xsl:param
	name="position"/>
    <!-- Determine logical number -->
    <xsl:variable
	name="ordinal"
	select="$start - 1 + $position"/>
    <xsl:choose>
      <xsl:when
	  test="$enumType = 'arabic'">
	<xsl:value-of
	    select="$ordinal"/>
      </xsl:when>
      <xsl:when
	  test="$enumType = 'loweralpha'">
	<xsl:value-of
	    select="substring('abcdefghijklmnopqrstzuvwxyz', $ordinal, 1)"/>
      </xsl:when>
      <xsl:when
	  test="$enumType = 'upperalpha'">
	<xsl:value-of
	    select="substring('ABCDEFGHIJKLMNOPQRSTZUVWXYZ', $ordinal, 1)"/>
      </xsl:when>
      <!-- TODO Support for counting roman numbers -->
      <xsl:when
	  test="$enumType = 'lowerroman'">
	<xsl:text>i</xsl:text>
      </xsl:when>
      <xsl:when
	  test="$enumType = 'upperroman'">
	<xsl:text>I</xsl:text>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!--
  Content Model: (title?, tgroup+)
  Attributes:    The table element contains the common attributes and:
  frame, colsep, rowsep, pgwide
  -->
  <!-- == compound_body_element -->
  <xsl:template
      match="table">
    <xsl:call-template
	name="u:outputClass"/>
    <xsl:call-template
	name="u:blank"/>
    <xsl:apply-templates
	select="tgroup"/>
    <xsl:if
	test="title">
      <!-- TODO A table title must be rendered by using the `.. table::'
                directive -->
      <xsl:call-template
	  name="u:BandI"/>
      <xsl:apply-templates
	  select="title"/>
      &tEOL;
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (colspec*, thead?, tbody)
  Attributes:    The tgroup element contains the common attributes and:
  cols, colsep, rowsep, align
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="tgroup">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: EMPTY
  Attributes:    The colspec element contains the common attributes and:
  colnum, colname, colwidth, colsep, rowsep, align, char, charoff

  The colwidth attribute gives the width of the respective column in characters
  including padding whitespace but no separator markup.
  -->
  <!-- == simple_body_subelement -->
  <!-- @colwidth needed by children but element has no own output -->
  <xsl:template
      match="colspec"/>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (row+)
  Attributes:    The thead element contains the common attributes and:
  valign
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="thead">
    <xsl:apply-templates/>
  </xsl:template>

  <!--
  Content Model: (row+)
  Attributes:    The tbody element contains the common attributes and:
  valign
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="tbody">
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (entry+)
  Attributes:    The row element contains the common attributes and:
  rowsep, valign
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="row">
    <!-- Separator line above unless first row of a tbody with no previous
         thead (in this case the separator line is output already as the
         closing separator line of thead) -->
    <xsl:if
	test="position() > 1 or parent::thead or parent::tbody and not(../../thead)">
      <xsl:call-template
	  name="u:rowSeparatorLine"/>
    </xsl:if>
    <!-- Determine heights in physical lines of all entries -->
    <xsl:variable
	name="heights">
      <xsl:for-each
	  select="entry">
	<xsl:variable
	    name="text">
	  <!-- Catch the text of all entries -->
	  <xsl:apply-templates/>
	</xsl:variable>
	<!-- Compute height of this entry; leading and trailing EOL must be
             subtracted -->
	<xsl:value-of
	    select="string-length($text) - string-length(translate($text, '&#xA;', '')) - 1"/>
	&tSP; <!-- A space as a list separator -->
      </xsl:for-each>
    </xsl:variable>
    <!-- Determine maximum height so every entry must be this high -->
    <xsl:variable
	name="maxHeight"
	select="math:max(str:tokenize(normalize-space($heights)))"/>
    <!-- Output all the physical lines of this row -->
    <xsl:call-template
	name="u:rowLines">
      <xsl:with-param
	  name="currentLine"
	  select="1"/>
      <xsl:with-param
	  name="maxLine"
	  select="$maxHeight"/>
    </xsl:call-template>
    <!-- Output final separator line if this is the last row -->
    <xsl:if
	test="position() = last()">
      <xsl:call-template
	  name="u:rowSeparatorLine">
	<xsl:with-param
	    name="char">
	  <!-- Determine correct character for the separator line -->
	  <xsl:choose>
	    <xsl:when
		test="parent::thead">
	      <xsl:value-of
		  select="'='"/>
	    </xsl:when>
	    <xsl:otherwise>
	      <xsl:value-of
		  select="'-'"/>
	    </xsl:otherwise>
	  </xsl:choose>
	</xsl:with-param>
      </xsl:call-template>
    </xsl:if>
  </xsl:template>

  <!-- Output physical line $currentLine of a row and continue until
       line $maxLine is output -->
  <xsl:template
      name="u:rowLines">
    <xsl:param
	name="currentLine"/>
    <xsl:param
	name="maxLine"/>
    <xsl:if
	test="$currentLine &lt;= $maxLine">
      <!-- If there are still physical lines to output do it -->
      <xsl:call-template
	  name="u:indent"/>
      <!-- Leading bar -->
      <xsl:text>|</xsl:text>
      <xsl:apply-templates>
	<xsl:with-param
	    name="currentLine"
	    select="$currentLine"/>
      </xsl:apply-templates>
      <!-- End of this physical line -->
      &tEOL;
      <!-- Continue with the next physical line -->
      <xsl:call-template
	  name="u:rowLines">
	<xsl:with-param
	    name="currentLine"
	    select="$currentLine + 1"/>
	<xsl:with-param
	    name="maxLine"
	    select="$maxLine"/>
      </xsl:call-template>
    </xsl:if>
  </xsl:template>

  <!-- Output a separator line with all the right characters -->
  <xsl:template
      name="u:rowSeparatorLine">
    <xsl:param
	name="char"
	select="'-'"/>
    <xsl:call-template
	name="u:indent"/>
    <xsl:text>+</xsl:text>
    <xsl:for-each
	select="../../colspec">
      <xsl:value-of
	  select="str:padding(@colwidth, $char)"/>
      <xsl:text>+</xsl:text>
    </xsl:for-each>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: (%body.elements;)*
  Attributes:    The entry element contains the common attributes and:
  colname, namest, morerows, colsep, rowsep, align, char, charoff, valign and
  morecols
  -->
  <!-- == compound_body_subelement -->
  <xsl:template
      match="entry">
    <!-- TODO `classes` attribute needs support -->
    <!-- This is called in two ways; if $currentLine = 0 all physical lines
         of this entry must be output; if $currentLine > 0 the physical line
         with exactly this number shall be output -->
    <xsl:param
	name="currentLine"
	select="0"/>
    <xsl:variable
	name="column"
	select="position() + sum(preceding-sibling::entry/@morecols)"/>
    <!-- Determine width in characters needed for this entry -->
    <xsl:variable
	name="width">
      <xsl:call-template
	  name="u:computeEntryWidth">
	<xsl:with-param
	    name="colspecs"
	    select="../../../colspec"/>
	<xsl:with-param
	    name="column"
	    select="$column"/>
	<xsl:with-param
	    name="span"
	    select="@morecols"/>
      </xsl:call-template>
    </xsl:variable>
    <!-- Output the entry completely or a certain physical line -->
    <xsl:call-template
	name="u:outputEntry">
      <xsl:with-param
	  name="string">
	<!-- Capture physical lines of the entry in a variable -->
	<xsl:apply-templates/>
      </xsl:with-param>
      <xsl:with-param
	  name="width"
	  select="$width"/>
      <xsl:with-param
	  name="expectedIndent">
	<!-- Capture indent for the entry generated by the normal template
             rules to remove it later -->
	<xsl:call-template
	    name="u:indent"/>
      </xsl:with-param>
      <xsl:with-param
	  name="outputLine"
	  select="$currentLine"/>
    </xsl:call-template>
    <!-- Final bar after the entry -->
    <xsl:text>|</xsl:text>
  </xsl:template>

  <!-- Compute width of the entry -->
  <xsl:template
      name="u:computeEntryWidth">
    <!-- The colspec elements of all columns -->
    <xsl:param
	name="colspecs"/>
    <!-- Column of this entry -->
    <xsl:param
	name="column"/>
    <!-- Number of columns this entry spans -->
    <xsl:param
	name="span"
	select="0"/>
    <xsl:param
	name="sum"
	select="0"/>
    <xsl:choose>
      <xsl:when
	  test="$span">
	<!-- If entry spans multiple columns compute their width -->
	<xsl:call-template
	    name="u:computeEntryWidth">
	  <xsl:with-param
	      name="colspecs"
	      select="$colspecs"/>
	  <xsl:with-param
	      name="column"
	      select="$column + 1"/>
	  <xsl:with-param
	      name="span"
	      select="$span - 1"/>
	  <!-- Add the separator character and the following column width -->
	  <xsl:with-param
	      name="sum"
	      select="$sum + 1 + $colspecs[$column]/@colwidth"/>
	</xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
	<!-- Does not span more columns so return sum and width of this
             column -->
	<xsl:value-of
	    select="$sum + $colspecs[$column]/@colwidth"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- Outputs one or all lines of a table entry as a string trimmed left and
       padded -->
  <xsl:template
      name="u:outputEntry">
    <!-- Width of the entry; there is no provision for actual physical lines
         longer than this width -->
    <xsl:param
	name="width"/>
    <!-- The string containing the remaining physical lines; may be an empty
         string -->
    <xsl:param
	name="string"
	select="''"/>
    <!-- The indendation which is expected to be prefixed before every
         physical line -->
    <xsl:param
	name="expectedIndent"
	select="''"/>
    <!-- Is this the first call to this template -->
    <xsl:param
	name="isFirst"
	select="true()"/>
    <!-- Number of physical line to output or 0 to output all lines -->
    <xsl:param
	name="outputLine"
	select="0"/>
    <!-- Output is wanted if all or the first physical line are to be
         output -->
    <xsl:variable
	name="doOutput"
	select="$outputLine = 0 or $outputLine = 1"/>
    <xsl:variable
	name="stringLFHalfTrimmed">
      <xsl:choose>
	<xsl:when
	    test="$isFirst and substring($string, 1, 1) = '&#x0A;'">
	  <!-- Remove leading linefeed if this is the first time -->
	  <xsl:value-of
	      select="substring($string, 2)"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="$string"/>
	  </xsl:otherwise>
      </xsl:choose>
    </xsl:variable>
    <xsl:variable
	name="stringLFTrimmed">
      <xsl:choose>
	<xsl:when
	    test="$isFirst and substring($stringLFHalfTrimmed, string-length($stringLFHalfTrimmed), 1) = '&#x0A;'">
	  <!-- Remove trailing linefeed if this is the first time -->
	  <xsl:value-of
	      select="substring($stringLFHalfTrimmed, 1, string-length($stringLFHalfTrimmed) - 1)"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="$stringLFHalfTrimmed"/>
	  </xsl:otherwise>
      </xsl:choose>
    </xsl:variable>
    <!-- Determine remaining lines after the first one -->
    <xsl:variable
	name="remainingLines">
      <xsl:if
	  test="contains($stringLFTrimmed, '&#x0A;')">
	<xsl:value-of
	    select="substring-after($stringLFTrimmed, '&#x0A;')"/>
      </xsl:if>
    </xsl:variable>
    <xsl:if
	test="$doOutput">
      <!-- If this physical line must be output determine the first physical
           line -->
      <xsl:variable
	  name="firstLine">
	<xsl:choose>
	  <xsl:when
	      test="string-length($remainingLines)">
	    <xsl:value-of
		select="substring-before($stringLFTrimmed, '&#x0A;')"/>
	  </xsl:when>
	  <xsl:otherwise>
	    <xsl:value-of
		select="$stringLFTrimmed"/>
	  </xsl:otherwise>
	</xsl:choose>
      </xsl:variable>
      <!-- Remove the leading indentation from the physical line which is
           brought there by the normal templates -->
      <xsl:variable
	  name="trimmed">
	<xsl:if
	    test="string-length($firstLine)">
	  <!-- Trim only non-empty lines -->
	  <xsl:value-of
	      select="substring-after($firstLine, $expectedIndent)"/>
	</xsl:if>
      </xsl:variable>
      <!-- Pad the line with a leading and a trailing space -->
      <xsl:variable
	  name="padded"
	  select="concat(' ', $trimmed, ' ')"/>
      <!-- Output the padded value -->
      <xsl:value-of
	  select="$padded"/>
      <!-- Fill up the width of the entry with spaces -->
      <xsl:if
	  test="$width - string-length($padded) &lt; 0">
	<xsl:message>
	  <xsl:text>WARNING: Table column too narrow (minimum: </xsl:text>
	  <xsl:value-of
	      select="string-length($padded)"/>
	  <xsl:text>)</xsl:text>
	  &tEOL;
	</xsl:message>
      </xsl:if>
      <xsl:value-of
	  select="str:padding($width - string-length($padded))"/>
    </xsl:if>
    <xsl:if
	test="$outputLine > 1 or $outputLine = 0 and string-length($remainingLines)">
      <!-- If a following physical line must be output or if all physical
           lines shall be output and there are remaining physical lines -->
      <xsl:if
	  test="$outputLine = 0">
	<!-- Output linefeed only if we output all the lines -->
	&tEOL;
      </xsl:if>
      <!-- Output the remaining lines -->
      <xsl:call-template
	  name="u:outputEntry">
	<xsl:with-param
	    name="width"
	    select="$width"/>
	<xsl:with-param
	    name="string"
	    select="$remainingLines"/>
	<xsl:with-param
	    name="expectedIndent"
	    select="$expectedIndent"/>
	<xsl:with-param
	    name="isFirst"
	    select="false()"/>
	<xsl:with-param
	    name="outputLine">
	  <xsl:choose>
	    <xsl:when
		test="$outputLine = 0">
	      <xsl:value-of
		  select="0"/>
	    </xsl:when>
	    <xsl:otherwise>
	      <xsl:value-of
		  select="$outputLine - 1"/>
	    </xsl:otherwise>
	  </xsl:choose>
	</xsl:with-param>
      </xsl:call-template>
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!-- == inline_element -->
  <xsl:template
      match="citation_reference">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>[</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>]_</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == inline_element -->
  <xsl:template
      match="emphasis">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>*</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>*</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- user-numbered footnotes lack @auto -->
  <!-- == inline_element -->
  <xsl:template
      match="footnote_reference[not(@auto)]">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>[</xsl:text>
    <xsl:value-of
	select="text()"/>
    <xsl:text>]_</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
    <!-- child paragraph provides blank line -->
  </xsl:template>

  <!-- automatically numbered footnotes have @auto -->
  <!-- if @names is different from label content, it is a named footnote -->
  <!-- == inline_element -->
  <xsl:template
      match="footnote_reference[@auto='1']">
    <xsl:variable
	name="ref"
	select="@refid"/>
    <xsl:if
	test="not(starts-with(//footnote[@ids=$ref]/@names, 'TARGET_NOTE:\ '))">
      <!-- Not a generated footnote reference for a `.. target-notes::';
           such footnote reference and the preceding space should be
           embedded in `generated'! -->
      <xsl:call-template
	  name="u:bkslshEscPre"/>
      <xsl:text>[#</xsl:text>
      <xsl:if
	  test="//footnote[@ids=$ref]/@names != //footnote[@ids=$ref]/label">
	<xsl:call-template
	    name="u:outputNames">
	  <xsl:with-param
	      name="names"
	      select="//footnote[@ids=$ref]/@names"/>
	</xsl:call-template>
      </xsl:if>
      <xsl:text>]_</xsl:text>
      <xsl:call-template
	  name="u:bkslshEscSuf"/>
    </xsl:if>
  </xsl:template>

  <!-- automatically symboled footnotes have @auto=* -->
  <!-- == inline_element -->
  <xsl:template
      match="footnote_reference[@auto='*']">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>[*]_</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  -->
  <!-- == inline_element -->
  <xsl:template
      match="literal">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>``</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>``</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Attribute combinations found in `standard' text and other examples:
       @refuri = standalone hyperlink
       @ids @refid = TOC, probably all in <generated>
       @name @refuri with matching <target> in document = named external hyperlink _
       @name @refuri immediately followed by matching <target> = named embedded URI _
       @name @refuri with no matching <target> in document = anonymous embedded URI __
       @anonymous @name @refuri with no matching <target> in document = anonymous explicit URI __
       @name @refid = internal cross-reference _
       @anonymous @name @refid = anonymous implicit internal reference __
       @name @refid image = clickable image to internal reference _
       @refuri image = clickable image to standalone hyperlink

       A target matches if target/@names contains the lower cased, whitespace
       quoted reference/@name
  -->

  <!-- Standalone hyperlink -->
  <!-- == inline_element -->
  <xsl:template
      match="reference[not(@name or @anonymous)]">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:choose>
      <xsl:when
	  test="starts-with(., 'PEP ')">
	<xsl:text>:PEP:`</xsl:text>
	<xsl:value-of
	    select="substring-after(., 'PEP ')"/>
	<xsl:text>`</xsl:text>
      </xsl:when>
      <xsl:when
	  test="starts-with(., 'RFC ')">
	<xsl:text>:RFC:`</xsl:text>
	<xsl:value-of
	    select="substring-after(., 'RFC ')"/>
	<xsl:text>`</xsl:text>
      </xsl:when>
      <xsl:otherwise>
	<xsl:apply-templates/>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- External references -->
  <!-- == inline_element -->
  <xsl:template
      match="reference[@name and @refuri]">
    <!-- Determine normalized name by downcasing it -->
    <xsl:variable
	name="normalized"
	select="translate(normalize-space(@name), 'ABCDEFGHIJKLMNOPQRSTUVWXYZ', 'abcdefghijklmnopqrstuvwxyz')"/>
    <xsl:variable
	name="quoted"
	select="str:replace($normalized, ' ', '\ ')"/>
    <xsl:variable
	name="matching"
	select="//target[contains(@names, $quoted)]"/>
    <xsl:call-template
	name="u:inlineReference">
      <xsl:with-param
	  name="anonymous"
	  select="not($matching) or @anonymous"/>
      <xsl:with-param
	  name="embedded"
	  select="not(@anonymous) and (not($matching) or generate-id(following-sibling::node()[1]) = generate-id($matching))"/>
    </xsl:call-template>
  </xsl:template>

  <!-- Internal references -->
  <!-- == inline_element -->
  <xsl:template
      match="reference[@name and @refid]">
    <xsl:call-template
	name="u:inlineReference">
      <xsl:with-param
	  name="anonymous"
	  select="@anonymous"/>
    </xsl:call-template>
  </xsl:template>

  <!-- Image references -->
  <!-- == inline_element -->
  <xsl:template
      match="reference[image]">
    <!-- All done by the `image' tag -->
    <xsl:apply-templates/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  -->
  <!-- == inline_element -->
  <xsl:template
      match="strong">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>**</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>**</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == inline_element -->
  <xsl:template
      match="subscript">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>:sub:`</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>`</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- == inline_element -->
  <xsl:template
      match="superscript">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>:sup:`</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>`</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- The target element has various roles depending on context; they are
       all handled here -->
  <!-- == simple_body_element == inline_element == directive -->
  <xsl:template
      match="target">
    <xsl:choose>
      <xsl:when
	  test="name(preceding-sibling::*[1]) = 'reference'">
        <!-- An embedded inline target - handled by the reference itself -->
      </xsl:when>
      <xsl:when
	  test="contains($inline_containers, concat('*', name(..), '*'))">
	<!-- An inline target of some sort -->
	<xsl:call-template
	    name="u:inlineReference">
	  <xsl:with-param
	      name="isTarget"
	      select="true()"/>
	</xsl:call-template>
      </xsl:when>
      <xsl:when
	  test="@anonymous">
	<!-- An anonymous target directive -->
	<xsl:call-template
	    name="u:outputClass"/>
	<xsl:call-template
	    name="u:BandI"/>
	<xsl:text>__ </xsl:text>
	<xsl:choose>
	  <xsl:when
	      test="@refid">
	    <xsl:call-template
		name="u:outputNamesRefid"/>
	    <xsl:text>_</xsl:text>
	  </xsl:when>
	  <xsl:when
	      test="@refuri">
	    <xsl:value-of
		select="@refuri"/>
	  </xsl:when>
	</xsl:choose>
	&tEOL;
      </xsl:when>
      <xsl:when
	  test="@names | @refid">
	<!-- A target directive -->
	<xsl:call-template
	    name="u:outputClass"/>
	<xsl:call-template
	    name="u:BandI"/>
	<xsl:text>.. _</xsl:text>
	<xsl:choose>
	  <xsl:when
	      test="@refid">
	    <xsl:call-template
		name="u:outputNamesRefid"/>
	  </xsl:when>
	  <xsl:otherwise>
	    <xsl:call-template
		name="u:outputNames"/>
	  </xsl:otherwise>
	</xsl:choose>
	<xsl:text>:</xsl:text>
	<xsl:if
	    test="@refuri">
	  <xsl:text> </xsl:text>
	  <xsl:value-of
	      select="@refuri"/>
	</xsl:if>
	&tEOL;
      </xsl:when>
      <xsl:otherwise>
	<!-- Should not happen -->
	<xsl:call-template
	    name="u:notSupported"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  -->
  <!-- == inline_element -->
  <xsl:template
      match="title_reference">
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>`</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>`</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Content Model: %text.model;
  -->
  <!-- == inline_element -->
  <xsl:template
      match="inline">
    <!-- TODO `role' directives must be generated for plain and derived
              user-defined roles. -->
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <xsl:text>:</xsl:text>
    <xsl:value-of
	select="@classes"/>
    <xsl:text>:`</xsl:text>
    <xsl:apply-templates/>
    <xsl:text>`</xsl:text>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- TODO `meta` directive must be implemented. -->

  <!-- ******************************************************************** -->

  <!--
  Docutils wraps generated elements around text that is inserted (generated) by
  Docutils; i.e., text that was not in the document, like section numbers 
  inserted by the "sectnum" directive.
  -->
  <!-- == inline_element -->
  <xsl:template
      match="generated"/>

  <!-- == inline_element -->
  <xsl:template
      match="problematic">
    <!-- Simply output the contained text because this is probably the
         original text-->
    <xsl:value-of
	select="text()"/>
  </xsl:template>

  <!-- == compound_body_element -->
  <xsl:template
      match="system_message"/>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!--
  When a block of text contains linefeeds, it was indented relative to a marker
  on the first line
  -->
  <xsl:template
      match="text()">
    <xsl:call-template
	name="u:indentLF"/>
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!-- Add a blank line if necessary and indent -->
  <xsl:template
      name="u:BandI">
    <xsl:call-template
	name="u:blank"/>
    <xsl:call-template
	name="u:indent"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Add a blank line in certain contexts -->
  <xsl:template
      name="u:blank">
    <xsl:apply-templates
        mode="blankSkipInline"
	select="preceding::*[1]"/>
  </xsl:template>

  <!-- Find the preceding element we are really interested in and act
       according to this element -->
  <xsl:template
      mode="blankSkipInline"
      match="*">
    <xsl:choose>
      <!-- Skip all inline elements and body subelements and check their
           parents -->
      <xsl:when
          test="contains(concat($inline_elements, $simple_body_subelements, $compound_body_subelements), concat('*', name(.), '*'))">
	<xsl:apply-templates
	    mode="blankSkipInline"
	    select=".."/>
      </xsl:when>
      <xsl:otherwise>
	<!-- Reached the type of element we decide on -->
	<xsl:if
	    test="contains($blank_after, concat('*', name(.), '*'))">
	  &tCR;
	</xsl:if>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!--
  Indent a block if it's a child of...
  -->
  <data:lookup>
    <node
	name="address"
	indent="10"/>
    <node
	name="author"
	indent="9"/>
    <node
	name="authors"
	indent="10"/>
    <node
	name="contact"
	indent="10"/>
    <node
	name="copyright"
	indent="12"/>
    <node
	name="date"
	indent="7"/>
    <node
	name="organization"
	indent="15"/>
    <node
	name="revision"
	indent="11"/>
    <node
	name="status"
	indent="9"/>
    <node
	name="version"
	indent="10"/>
    <!-- This is only for `bullet_list/list_item';
         `enumerated_list/list_item' is handled special -->
    <node
	name="list_item"
	indent="2"/>
    <node
	name="definition_list_item"
	indent="4"/>
    <node
	name="field_body"
	indent="4"/>
    <node
	name="option_list_item"
	indent="4"/>
    <!-- This is also the indentation if block_quote comes as one of the
         special directives -->
    <node
	name="block_quote"
	indent="4"/>
    <node
	name="literal_block"
	indent="4"/>
    <node
	name="attribution"
	indent="3"/>
    <node
	name="line"
	indent="2"/>
  </data:lookup>

  <!-- Do indent according to ancestor -->
  <xsl:template
      name="u:indent">
    <!-- In some cases the ancestors to indent for need to be determined
         by the calling template -->
    <xsl:param
	name="ancestors"
	select="ancestor::*"/>
    <xsl:for-each
	select="$ancestors">
      <xsl:variable
	  name="this"
	  select="name()"/>
      <xsl:choose>
	<xsl:when
	    test="contains($directives, concat('*', $this, '*'))">
	  <!-- TODO Indentation of lines after some directives must be
	            indented to align with the directive instead of a
	            fixed indentation; however, this is rather complicated
	            since identation for parameters should be fixed -->
	  <xsl:value-of
	      select="str:padding(3)"/>
	</xsl:when>
	<xsl:when
	    test="$this = 'list_item' and parent::enumerated_list">
	  <!-- Enumerated list items base their indentation on the
               numeration -->
	  <xsl:variable
	      name="enumerator">
	    <xsl:call-template
		name="u:outputEnumerator"/>
	  </xsl:variable>
	  <xsl:value-of
	      select="str:padding(string-length($enumerator))"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="str:padding(document('')//data:lookup/node[@name=$this]/@indent)"/>
	</xsl:otherwise>
      </xsl:choose>
    </xsl:for-each>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Indent a text containing line feeds correctly -->
  <xsl:template
      name="u:indentLF">
    <xsl:param
	name="string"
	select="."/>
    <!-- A fixed indentation may be given by caller -->
    <xsl:param
	name="indent">
      <!-- If not given compute it -->
      <xsl:call-template
	  name="u:indent"/>
    </xsl:param>
    <xsl:choose>
      <xsl:when
	  test="contains($string,'&#x0A;')">
	<!-- Output first physical line -->
	<xsl:value-of
	    select="substring-before($string, '&#x0A;')"/>
	&tEOL;
	<!-- Indent before the next line -->
	<xsl:value-of
	    select="$indent"/>
	<!-- Output remaining physical lines -->
	<xsl:call-template
	    name="u:indentLF">
	  <xsl:with-param
	      name="string"
	      select="substring-after($string, '&#x0A;')"/>
	  <xsl:with-param
	      name="indent"
	      select="$indent"/>
	</xsl:call-template>
      </xsl:when>
      <!-- String does not contain newline, so just output it -->
      <xsl:otherwise> 
	<xsl:value-of
	    select="$string"/>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Do output for those elements which do fold their inline content -->
  <xsl:template
      name="u:outputFolding">
    <!-- The prefix text to be output before the body -->
    <xsl:param
	name="prefix"
	select="''"/>
    <!-- The indentation for this body -->
    <xsl:param
	name="indent">
      <xsl:call-template
	  name="u:indent">
	<xsl:with-param
	    name="ancestors"
	    select="ancestor-or-self::*"/>
      </xsl:call-template>
    </xsl:param>
    <xsl:variable
	name="string">
      <!-- TODO Whitespace count of inline literals must be preserved -->
      <xsl:apply-templates/>
    </xsl:variable>
    <!-- Always output prefix with all trailing and leading spaces -->
    <xsl:value-of
	select="$prefix"/>
    <xsl:choose>
      <xsl:when
	  test="$fold &gt; 0">
	<xsl:variable
	    name="normalized"
	    select="normalize-space($string)"/>
	<xsl:choose>
	  <xsl:when
	      test="$string = ''">
	    <!-- Empty strings need no output -->
	  </xsl:when>
	  <xsl:when
	      test="$normalized = ''">
	    <!-- Only white-space in string; output a single space here -->
	    &tSP;
	  </xsl:when>
	  <xsl:otherwise>
	    <!-- Output leading white-space here -->
	    <xsl:if
		test="normalize-space(substring($string, 1, 1)) = ''">
	      &tSP;
	    </xsl:if>
	    <xsl:call-template
		name="u:indentFold">
	      <xsl:with-param
		  name="string"
		  select="$normalized"/>
	      <xsl:with-param
		  name="indent"
		  select="$indent"/>
	      <xsl:with-param
		  name="cursorColumn"
		  select="string-length($indent) + string-length($prefix)"/>
	    </xsl:call-template>
	    <!-- Output trailing white-space here -->
	    <xsl:if
		test="normalize-space(substring($string, string-length($string), 1)) = ''">
	      &tSP;
	    </xsl:if>
	  </xsl:otherwise>
	</xsl:choose>
      </xsl:when>
      <xsl:otherwise>
	<xsl:value-of
	    select="$string"/>
      </xsl:otherwise>
    </xsl:choose>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Indent a string with folding -->
  <xsl:template
      name="u:indentFold">
    <!-- Normalized string to output -->
    <xsl:param
	name="string"/>
    <!-- Indentation to use for a new line -->
    <xsl:param
	name="indent"/>
    <!-- Current output column -->
    <!-- TODO This is not a correct assumption for field definitions where
              the field name effectively determines the column of the first
              line -->
    <xsl:param
	name="cursorColumn"
	select="string-length($indent)"/>
    <!-- Do we start on a new (indented) line? -->
    <xsl:param
	name="isNewLine"
	select="true()"/>
    <xsl:variable
	name="firstWord">
      <xsl:choose>
	<!-- TODO Quoted spaces must not end a word -->
	<xsl:when
	    test="contains($string, ' ')">
	  <xsl:value-of
	      select="substring-before($string, ' ')"/>
	</xsl:when>
	<xsl:otherwise>
	  <xsl:value-of
	      select="$string"/>
	</xsl:otherwise>
      </xsl:choose>
    </xsl:variable>
    <xsl:variable
	name="rest"
	select="substring-after($string, ' ')"/>
    <xsl:choose>
      <xsl:when
	  test="$string = ''"/>
      <xsl:when
	  test="$isNewLine">
	<!-- Output at least first word -->
	<xsl:value-of
	    select="$firstWord"/>
	<xsl:call-template
	    name="u:indentFold">
	  <xsl:with-param
	      name="string"
	      select="$rest"/>
	  <xsl:with-param
	      name="indent"
	      select="$indent"/>
	  <xsl:with-param
	      name="cursorColumn"
	      select="$cursorColumn + string-length($firstWord)"/>
	  <xsl:with-param
	      name="isNewLine"
	      select="false()"/>
	</xsl:call-template>
      </xsl:when>
      <xsl:when
	  test="$cursorColumn + 1 + string-length($firstWord) &gt; $fold">
	<!-- Line would get too long; start new line, indent and continue -->
	&tEOL;
	<xsl:value-of
	    select="$indent"/>
	<xsl:call-template
	    name="u:indentFold">
	  <xsl:with-param
	      name="string"
	      select="$string"/>
	  <xsl:with-param
	      name="indent"
	      select="$indent"/>
	  <xsl:with-param
	      name="cursorColumn"
	      select="string-length($indent)"/>
	  <xsl:with-param
	      name="isNewLine"
	      select="true()"/>
	</xsl:call-template>
      </xsl:when>
      <xsl:otherwise>
	<!-- In a line and first word fits; separate and add word -->
	&tSP;
	<xsl:value-of
	    select="$firstWord"/>
	<xsl:call-template
	    name="u:indentFold">
	  <xsl:with-param
	      name="string"
	      select="$rest"/>
	  <xsl:with-param
	      name="indent"
	      select="$indent"/>
	  <xsl:with-param
	      name="cursorColumn"
	      select="$cursorColumn + 1 + string-length($firstWord)"/>
	  <xsl:with-param
	      name="isNewLine"
	      select="false()"/>
	</xsl:call-template>
      </xsl:otherwise>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output attributes of the current element as a field list -->
  <xsl:template
      name="u:params">
    <xsl:param
	name="params"
	select="@*"/>
    <!-- Ancestors are needed for determining indentation; caller may give
         them -->
    <xsl:param
	name="ancestors"
	select="ancestor-or-self::*"/>
    <xsl:for-each
	select="$params">
      <!-- Skip URIs based on parent -->
      <xsl:if
	  test="name() != 'uri' and name() != 'xml:space'">
	<xsl:call-template
	    name="u:param">
	  <xsl:with-param
	      name="ancestors"
	      select="$ancestors"/>
	</xsl:call-template>
      </xsl:if>
    </xsl:for-each>
  </xsl:template>

  <!-- Output one attribute of the current element as a field list -->
  <xsl:template
      name="u:param">
    <xsl:param
	name="name"
	select="name()"/>
    <xsl:param
	name="value"
	select="."/>
    <!-- Ancestors are needed for determining indentation; caller may give
         them -->
    <xsl:param
	name="ancestors"
	select="ancestor::*"/>
    <xsl:call-template
	name="u:indent">
      <xsl:with-param
	  name="ancestors"
	  select="$ancestors"/>
    </xsl:call-template>
    <xsl:text>:</xsl:text>
    <xsl:choose>
      <xsl:when
	  test="$name = 'classes'">
	<xsl:text>class</xsl:text>
      </xsl:when>
      <xsl:otherwise>
	<xsl:value-of
	    select="$name"/>
      </xsl:otherwise>
    </xsl:choose>
    <xsl:text>:</xsl:text>
    <xsl:if
	test="$value">
      <xsl:text> </xsl:text>
      <xsl:value-of
	  select="$value"/>
    </xsl:if>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output `\' or `\ ' before some inline element if necessary -->
  <xsl:template
      name="u:bkslshEscPre">
    <!-- Get the sibling node directly before the current element -->
    <xsl:variable
	name="before"
	select="preceding-sibling::node()[1]"/>
    <xsl:choose>
      <!-- No sibling before this node -->
      <xsl:when
	  test="not($before)"/>
      <!-- Element directly before this - must be another inline element -->
      <xsl:when
	  test="name($before)">
	<!-- So separate it by a quoted space -->
	<xsl:text>\ </xsl:text>
      </xsl:when>
      <!-- Node directly before this is text - check it -->
      <xsl:when
	  test="not(contains(concat($apos, ' &#xA;&#x9;&#xD;&quot;([{&lt;-/:'), substring($before, string-length($before), 1)))">
	<!-- Does not end in one of the allowed characters so separate it -->
	<xsl:text>\ </xsl:text>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

  <!-- Output `\' after some inline element if necessary -->
  <xsl:template
      name="u:bkslshEscSuf">
    <!-- Get the sibling node directly after the current element -->
    <xsl:variable
	name="after"
	select="following-sibling::node()[1]"/>
    <xsl:choose>
      <!-- No sibling after this node -->
      <xsl:when
	  test="not($after)"/>
      <!-- Element directly after this - must be another inline element
           handling itself -->
      <xsl:when
	  test="name($after)"/>
      <!-- Node directly after this is text - check it -->
      <xsl:when
	  test="not(contains(concat($apos, ' &#xA;&#x9;&#xD;&quot;)]}&gt;-/:.,;!?\'), substring($after, 1, 1)))">
	<!-- Does not start with one of the allowed characters so separate
             it -->
	<xsl:text>\</xsl:text>
      </xsl:when>
    </xsl:choose>
  </xsl:template>

  <!-- ******************************************************************** -->

  <xsl:template
      name="u:notSupported">
    <xsl:call-template
	name="u:BandI"/>
    <xsl:text>######## NOT SUPPORTED: `</xsl:text>
    <xsl:value-of
	select="name(.)"/>
    <xsl:text>' ########</xsl:text>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <xsl:template
      name="u:overline">
    <!-- Length of the rendered(!) text -->
    <xsl:param
	name="length"/>
    <!-- Depth 1 and 2 are document title and subtitle while depths
         greater than 2 are normal section titles -->
    <xsl:param
	name="depth"/>
    <!-- Test whether a overline is wanted at all -->
    <xsl:if
	test="substring($adornment, 2 * ($depth - 1) + 1, 1) = 'o'">
      <xsl:value-of
	  select="str:padding($length, substring($adornment, 2 * ($depth - 1) + 2, 1))"/>
      &tEOL;
    </xsl:if>
  </xsl:template>

  <xsl:template
      name="u:underline">
    <!-- Length of the rendered(!) text -->
    <xsl:param
	name="length"/>
    <!-- Depth 1 and 2 are document title and subtitle while depths
         greater than 2 are normal section titles -->
    <xsl:param
	name="depth"/>
    <xsl:value-of
	select="str:padding($length, substring($adornment, 2 * ($depth - 1) + 2, 1))"/>
    &tEOL;
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output a non-standalone reference or target -->
  <xsl:template
      name="u:inlineReference">
    <xsl:param
	name="anonymous"
	select="false()"/>
    <xsl:param
	name="embedded"
	select="false()"/>
    <!-- Is this a target instead of a reference? -->
    <xsl:param
	name="isTarget"
	select="false()"/>
    <xsl:param
	name="text"
	select="node()"/>
    <xsl:call-template
	name="u:bkslshEscPre"/>
    <!-- Delimiter only if link contains other than alphanumerics -->
    <xsl:variable
	name="delimiter">
      <xsl:if
	  test="* or translate($text, '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz', '') or $embedded">
	<xsl:text>`</xsl:text>
      </xsl:if>
    </xsl:variable>
    <xsl:if
	test="$isTarget">
      <xsl:text>_</xsl:text>
    </xsl:if>
    <xsl:value-of
	select="$delimiter"/>
    <xsl:apply-templates
	select="$text"/>
    <xsl:if
	test="$embedded">
      <xsl:text> &lt;</xsl:text>
      <xsl:value-of
	  select="@refuri"/>
      <xsl:text>&gt;</xsl:text>
    </xsl:if>
    <xsl:value-of
	select="$delimiter"/>
    <xsl:if
	test="not($isTarget)">
      <xsl:text>_</xsl:text>
      <xsl:if
	  test="$anonymous">
	<xsl:text>_</xsl:text>
      </xsl:if>
    </xsl:if>
    <xsl:call-template
	name="u:bkslshEscSuf"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output a class directive for the directly following element. -->
  <!-- TODO A class directive can also be used as a container putting the
	    respective attribute to its content; however, this is not
	    reflected in XML - you'd need to check a sequence of elements
	    whether they all have the same attribute; furthermore class
	    settings for block quotes needs to be treated special -->
  <xsl:template
      name="u:outputClass">
    <!-- Blank line already output? -->
    <xsl:param
	name="alreadyBlanked"
	select="false()"/>
    <!-- Indentation already output? -->
    <xsl:param
	name="alreadyIndented"
	select="false()"/>
    <!-- Add a blank line after class directive? -->
    <xsl:param
	name="blankAfter"
	select="false()"/>
    <xsl:if
	test="@classes">
      <xsl:if
	  test="not($alreadyBlanked)">
	<xsl:call-template
	    name="u:blank"/>
      </xsl:if>
      <xsl:if
	  test="not($alreadyIndented)">
	<xsl:call-template
	    name="u:indent"/>
      </xsl:if>
      <xsl:text>.. class:: </xsl:text>
      <xsl:value-of
	  select="@classes"/>
      &tEOL;
      <xsl:if
	  test="$blankAfter">
	&tCR;
      </xsl:if>
    </xsl:if>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output a names attribute at index considering quoted spaces. -->
  <xsl:template
      name="u:outputNames">
    <!-- Blank line already output? -->
    <xsl:param
	name="names"
	select="@names"/>
    <xsl:param
	name="index"
	select="0"/>
    <xsl:value-of
	select="str:replace(str:tokenize(normalize-space(str:replace($names, '\ ', '|')))[position() = $index + 1], '|', ' ')"/>
  </xsl:template>

  <!-- ******************************************************************** -->

  <!-- Output a names attribute for a refid. -->
  <xsl:template
      name="u:outputNamesRefid">
    <xsl:param
	name="refid"
	select="@refid"/>
    <!-- Determine the elements which is referred -->
    <xsl:variable
	name="refElem"
	select="//*[@ids and math:max(dyn:map(str:tokenize(normalize-space(@ids)), 'number($refid = .)')) > 0]"/>
    <xsl:call-template
	name="u:outputNames">
      <xsl:with-param
	  name="names"
	  select="$refElem/@names"/>
      <xsl:with-param
	  name="index"
	  select="math:max(dyn:map(str:tokenize(normalize-space($refElem/@ids)), 'number($refid = .) * position() - 1'))"/>
    </xsl:call-template>
  </xsl:template>

  <!-- ******************************************************************** -->
  <!-- ******************************************************************** -->

  <!-- Report unknown tags -->
  <xsl:template
      match="*">
    <xsl:message>
      <xsl:text>`</xsl:text>
      <xsl:value-of
	  select="name(.)"/>
      <xsl:text>' encountered</xsl:text>
      <xsl:if
	  test="parent::*">
	<xsl:text> in `</xsl:text>
	<xsl:value-of
	    select="name(parent::*)"/>
	<xsl:text>'</xsl:text>
      </xsl:if>
      <xsl:text>, but no template matches.</xsl:text>
    </xsl:message>
  </xsl:template>

</xsl:stylesheet>

<!-- ********************************************************************** -->
<!-- ********************************************************************** -->
<!-- ********************************************************************** -->
<!-- POD manual page

=head1 NAME

xml2rst.xsl - An XSLT script to convert Docutils XML to reStructuredText

=head1 SYNOPSIS

  Xalan docutils.xml xml2rst.xsl

=head1 DESCRIPTION

B<xml2rst.xsl> is an XSLT script to convert Docutils XML to
reStructuredText. You can use your favorite XSLT processor supporting
EXSLT (e.g. xsltproc from the Gnome project) to generate
reStructuredText from the Docutils intermediate XML representation.
Its main use is to generate reStructuredText from some other format
where a converter to Docutils XML already exists.

=head2 Options

The following options are supported. They are XSLT parameters for the
whole script and must be given to the XSLT processor by the respective
option (Xalan: B<-p>).

=over 4

=item adornment

Configures title markup to use so different styles can be requested
easily.

The value of the parameter must be a string made up of a sequence of
character pairs. The first character of a pair is C<o> (overline) or
C<u> (underline) and the second character is the character to use for
the markup.

The first and the second character pair is used for document title and
subtitle, the following pairs are used for section titles where the
third pair is used for the top level section title.

Defaults to C<o=o-u=u-u~u:u.u`>.

=item fold

Configures whether long text lines in paragraphs should be folded and
to which length. This option is for input not coming from reST which
may have no internal line feeds in plain text strings.

If folding is enabled text strings not in a line feed preserving
context are first white-space normalized and then broken according to
the folding rules. Folding rules put out the first word and continue
to do so with the following words unless the next word would cross
the folding boundary. Words are delimited by white-space.

Defaults to C<0>, i.e. no folding.

=back

=head2 Unsupported features

It is generally not possible to create an exact reproduction of an
original reStructuredText source from an intermediate XML file. The
reason is that Docutils transports pretty much but not all information
of the original source into the XML. Also the sequence is changed
sometimes.

However, the coverage of Docutils features of B<xml2rst.xsl> is pretty
good. A few minor features are not supported:

=over 4

=item * Fully minimized style for literal blocks

=item * Substitution references for C<replace::> substitutions

=item * Counting roman numbers in enumerated lists

=item * Special table types like C<list-table::> and C<csv-table::>

=item * Custom role definitions

=back

=head1 INSTALLATION

Installation is not necessary. Just use the file as downloaded with
your favorite XSLT processor supporting EXSLT. For instance you can
use B<xsltproc> from the Gnome project.

=head1 AUTHOR

Stefan Merten <smerten@oekonux.de> based on works by David Priest.

=head1 LICENSE

This program is licensed under the terms of the GPL. See

	http://www.gnu.org/licenses/gpl.txt

=head1 AVAILABILTY

See

	http://www.merten-home.de/FreeSoftware/xml2rst/

=cut

-->
