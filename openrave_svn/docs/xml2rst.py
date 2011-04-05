#! /usr/bin/env python
# -*- coding: iso-8859-1 -*-

# Based on sample.py,v 4.1.2.6 2006/04/14 13:59:26 cvs Exp

# Copyright (C) 2009 Stefan Merten

# xml2rst.py is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published
# by the Free Software Foundation; either version 2 of the License,
# or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
# 02111-1307, USA.

"""
Convert a docutils XML file to reStructuredText syntax.

Do

	perldoc xml2rst.py

for a man page.
"""

"""
=head1 NAME

xml2rst.py -- convert a docutils XML file to reStructuredText syntax

=head1 SYNOPSIS

B<xml2rst.py> [B<-v>] I<xml> [I<reST>]

B<xml2rst.py> B<--help>

=head1 DESCRIPTION

Converts a docutils XML input file to reStructuredText source.

This can be used to transform another format to reStructuredText given you have
a transformation to docutils XML.

=cut
"""

###############################################################################
###############################################################################
# Import

import sys
import os.path
import re

from optparse import OptionParser, OptionGroup, OptionValueError, Option
from copy import copy

try:
    from lxml import etree
except ImportError:
    errorExit(2, ( "Python package 'lxml' is not available",
                   "You may try to use 'xml2rst.xsl' with a standalone XSLT processor like 'xalan' or 'xsltproc'", ))

###############################################################################
###############################################################################
# Constants

"""
@var MainXsltNm: Name of the main XSLT source file
@type MainXsltNm: str
"""
MainXsltNm = "xml2rst.xsl"

"""
@var ScriptNm: Name of the script
@type ScriptNm: str
"""
ScriptNm = sys.argv[0]

###############################################################################
###############################################################################
# Variables

"""
@var options: Options given on the command line
@type options: optparse.Values
"""
global options

###############################################################################
###############################################################################
# General functions

def pod2Head(pod):
    """
    @param pod: Snippet in POD format to be analyzed.
    @type pod: str

    @return: String of first `=headX' entry in POD snippet or empty string if
             none found.
    @rtype: str
    """
    for line in pod.split("\n"):
        if line.startswith("=head"):
            return line[len("=headX"):].strip()
    return ""

###############################################################################

def pod2Description(pod):
    """
    @param pod: Snippet in POD format to be analyzed.
    @type pod: str

    @return: Stripped text from all lines not being a POD line command.
    @rtype: str
    """
    result = ""
    for line in pod.split("\n"):
        if not line.startswith("="):
            result = result.strip() + " " + line.strip()
    return result.strip()

###############################################################################

def pod2OptionList(pod):
    """
    Return option names found in POD snippet. Option names are recognized in
    `=item B<option>' constructs.

    @param pod: Snippet in POD format to be analyzed.
    @type pod: str

    @return: All option names contained in POD snippet as a list.
    @rtype: [ str, ..., ]
    """
    result = [ ]
    for line in pod.split("\n"):
        found = re.search("^=item\s*B<(-[^>]+)>", line)
        if found:
            result.append(found.group(1))
    return result

###############################################################################

def pod2OptionKeywords(pod):
    """
    Return a dict mapping `OptionParser.add_option' keywords to values found in
    POD snippet.

    @param pod: Snippet in POD format to be analyzed.
    @type pod: str

    @return: Mapping for all values found. Currently `help' and `dest' are
             filled.
    @rtype: { keyword: value, ..., }
    """
    result = { 'help': "", }
    for line in pod.split("\n"):
        if line.startswith("=cut"):
            break
        found = re.search("^=item\s*B<--?([^>]+)>(?:=|\s*)", line)
        if found:
            result['help'] = ""
            optionName = found.group(1)
            found = re.search("I<([^>]+)>", line)
            if found:
                result['dest'] = found.group(1)
            elif len(optionName) > 1:
                result['dest'] = optionName
        else:
            result['help'] += line + "\n"
    result['help'] = result['help'].strip()
    if result.has_key('dest'):
        result['dest'] = result['dest'].replace("-", "_")
    else:
        errorExit(1, ( "Internal error: Missing `dest' in documentation string:",
                       pod, ))
    return result

###############################################################################

def pod2Argument(pod):
    """
    Return a list of two strings for `OptionGroup.__init__' describing the
    argument found in POD snippet.

    @param pod: Snippet in POD format to be analyzed.
    @type pod: str

    @return: Name of the argument and its description.
    @rtype: [ argument, description, ]
    """
    argument = ""
    description = ""
    for line in pod.split("\n"):
        if line.startswith("=cut"):
            break
        found = re.search("^=item\s*I<([^>]+)>", line)
        if found:
            description = ""
            argument = found.group(1)
        else:
            description += line + "\n"
    description = description.strip()
    return [ argument, description, ]

###############################################################################
def setDefaultOptions():
    global options
    class DummyOptions:
        pass
    options = DummyOptions()
    options.adornment = None
    options.version = False
    options.fold = None

def parseOptions(args=None):
    """
    Sets options and returns arguments.

    @return: Name of input file and optionally of output file.
    @rtype: ( str, [str,] )
    """
    global options
    pod = """

=head1 OPTIONS

=cut
    """
    optionParser = OptionParser("usage: %prog [option]... <xml> [<rst>]")

    pod = """

=head2 General options

=over 4

=cut
    """
    generalGroup = OptionGroup(optionParser, pod2Head(pod),
                               pod2Description(pod))

    pod = """

=item B<-a> I<adornment>

=item B<--adornment>=I<adornment>

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

=cut
    """
    generalGroup.add_option(default=None, *pod2OptionList(pod),
                            **pod2OptionKeywords(pod))

    pod = """

=item B<-f> I<fold>

=item B<--fold>=I<fold>

Configures whether long text lines in paragraphs should be folded and
to which length. This option is for input not coming from reST which
may have no internal line feeds in plain text strings.

If folding is enabled text strings not in a line feed preserving
context are first white-space normalized and then broken according to
the folding rules. Folding rules put out the first word and continue
to do so with the following words unless the next word would cross
the folding boundary. Words are delimited by white-space.

Defaults to C<0>, i.e. no folding.

=cut
    """
    generalGroup.add_option(type="int", default=None,
                            *pod2OptionList(pod), **pod2OptionKeywords(pod))

    pod = """

=item B<-v>

=item B<--verbose>

Operate verbose.

=cut
    """
    generalGroup.add_option(action="store_true",
                            *pod2OptionList(pod), **pod2OptionKeywords(pod))
    optionParser.add_option_group(generalGroup)

    pod = """

=back

=head2 Arguments

=over 4

=cut
    """
    argumentGroup = OptionGroup(optionParser, pod2Head(pod),
                                pod2Description(pod))
    optionParser.add_option_group(argumentGroup)

    pod = """

=item I<xml>

The XML input file containing docutils XML.

=cut
    """

    argument1Group = OptionGroup(optionParser, *pod2Argument(pod))
    optionParser.add_option_group(argument1Group)

    pod = """

=item I<rst>

The optional output file containing reStructuredText.

If not given output is put to C<STDOUT>.

=cut
    """
    argument2Group = OptionGroup(optionParser, *pod2Argument(pod))
    optionParser.add_option_group(argument2Group)

    pod = """

=back

=cut
    """
    ( options, args, ) = optionParser.parse_args(args=args)

    if len(args) < 1:
        optionParser.error("An input file is required")
    if len(args) > 2:
        optionParser.error("At most two arguments are allowed")
    if (options.adornment is not None
        and re.search('^([ou][]!"#$%&\'()*+,\-./:;<=>?@[\\^_`{|}~])+$',
                      options.adornment) is None):
        optionParser.error("Invalid adornment string given")

    return args

###############################################################################

def errorOut(lines):
    """
    Outputs messages as error.

    @param lines: Messages to be output as single lines.
    @type lines: ( str, ..., )

    @return: 0
    @rtype: int
    """
    scriptName = os.path.basename(sys.argv[0])
    for line in lines:
        print >>sys.stderr, ("%s: %s" % ( scriptName, line, ))
    return 0

###############################################################################

def verboseOut(lines):
    """
    Outputs messages as a verbose message.

    @param lines: Messages to be output as single lines.
    @type lines: ( str, ..., )

    @return: 0
    @rtype: int
    """
    if options.verbose:
        errorOut([ "## " + line
                   for line in lines ])
    return 0

###############################################################################

def errorExit(code, lines):
    """
    Exit program with an error message.

    @param code: Exit Code to use.
    @type code: int

    @param lines: Strings to output as error message.
    @type lines: ( str, ..., )

    @return: Does not return.
    """
    errorOut(lines)
    sys.exit(code)

###############################################################################
###############################################################################
# Specialized functions

def convert(inNm=None, outNm=None,inF=None,outF=None,mainXsltF=None):
    """
    Do the conversion.

    @param inNm: Filename of input file.
    @type inNm: str

    @param outNm: Filename of output file or None.
    @type outNm: str | None
    """
    inFclose = False
    try:
        if inF is None:
            inF = open(inNm)
            inFclose = True
    except IOError:
        errorExit(1, ( "Can't open input file %r" % ( inNm, ), ))

    mainXsltFclose = False
    if mainXsltF is None:
        scriptP = os.path.dirname(os.path.realpath(ScriptNm))
        mainXsltNm = os.path.join(scriptP, MainXsltNm)
        try:
            mainXsltF = open(mainXsltNm)
            mainXsltFclose = True
        except IOError:
            errorExit(1, ( "Can't open main XSLT file %r" % ( mainXsltNm, ), ))
    else:
        mainXsltF.seek(0)

    xsltParser = etree.XMLParser()
    mainXsltDoc = etree.parse(mainXsltF, xsltParser)
    if mainXsltFclose:
        mainXsltF.close()
    mainXslt = etree.XSLT(mainXsltDoc)

    inParser = etree.XMLParser()
    try:
        inDoc = etree.parse(inF, inParser)
    except Exception, e:
        errorExit(1, ( "Error parsing input file %r: %s" % ( inNm, e, ), ))
    if inFclose:
        inF.close()

    xsltParams = { }
    if options.fold is not None:
        xsltParams['fold'] = str(options.fold)
    if options.adornment is not None:
        xsltParams['adornment'] = "'" + options.adornment + "'"
    try:
        result = mainXslt(inDoc, **xsltParams)
    except Exception, e:
        errorExit(1, ( "Error transforming input file %r: %s" % ( inNm, e, ), ))
    # Chop off trailing linefeed - added somehow
    outS = str(result)[:-1]
    if outNm is not None:
        try:
            outF = open(outNm, "w")
            outF.write(outS)
            outF.close()
        except IOError:
            errorExit(1, ( "Can't open output file %r" % ( outNm, ), ))
    elif outF is not None:
        outF.write(outS)
    else:
        print(outS)

###############################################################################
###############################################################################
# Classes

########################################################################
##############################################################################
# Now work

if __name__ == '__main__':
    arguments = parseOptions()
    inF = arguments[0]
    if len(arguments) > 1:
        outF = arguments[1]
    else:
        outF = None
    convert(inF, outF)

##############################################################################
##############################################################################

# TODO Accept additional XSLT sheets to create a transformation pipeline

# TODO Move from XSLT to Python implementation step by step by replacing
#      XSLT-code by Python code through extensions and other means

