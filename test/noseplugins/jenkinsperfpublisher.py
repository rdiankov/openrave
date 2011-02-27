"""This plugin provides test results to the Jenkins PerfPublisher XML format.

It was designed for the `Hudson`_ continuous build system but will
probably work for anything else that understands an XUnit-formatted XML
representation of test results.

    nosetests --with-jenkinsperf

And by default a file named jenkinsperf.xml will be written to the
working directory.

If you need to change the name or location of the file, you can set the
``--jenkinsperf-file`` option.

"""

import doctest
import os
import traceback
import re
import inspect
import platform
from nose.plugins.base import Plugin
from nose.exc import SkipTest
from time import time
from xml.sax import saxutils
from nose.pyversion import UNICODE_STRINGS
from distutils import ccompiler

import multiprocessing
globaljenkinsmanager = multiprocessing.Manager()
globaljenkinsstream = globaljenkinsmanager.list() # used for gathering statistics
globalxunitstats = multiprocessing.Array('i',[0]*4)

# Invalid XML characters, control characters 0-31 sans \t, \n and \r
CONTROL_CHARACTERS = re.compile(r"[\000-\010\013\014\016-\037]")

def xml_safe(value):
    """Replaces invalid XML characters with '?'."""
    return CONTROL_CHARACTERS.sub('?', value)

def escape_cdata(cdata):
    """Escape a string for an XML CDATA section."""
    return xml_safe(cdata).replace(']]>', ']]>]]&gt;<![CDATA[')

def nice_classname(obj):
    """Returns a nice name for class object or class instance.

        >>> nice_classname(Exception()) # doctest: +ELLIPSIS
        '...Exception'
        >>> nice_classname(Exception) # doctest: +ELLIPSIS
        '...Exception'

    """
    if inspect.isclass(obj):
        cls_name = obj.__name__
    else:
        cls_name = obj.__class__.__name__
    mod = inspect.getmodule(obj)
    if mod:
        name = mod.__name__
        # jython
        if name.startswith('org.python.core.'):
            name = name[len('org.python.core.'):]
        return "%s.%s" % (name, cls_name)
    else:
        return cls_name

def exc_message(exc_info):
    """Return the exception's message."""
    exc = exc_info[1]
    if exc is None:
        # str exception
        result = exc_info[0]
    else:
        try:
            result = str(exc)
        except UnicodeEncodeError:
            try:
                result = unicode(exc)
            except UnicodeError:
                # Fallback to args as neither str nor
                # unicode(Exception(u'\xe6')) work in Python < 2.6
                result = exc.args[0]
    return xml_safe(result)

class JenkinsPerfPublisher(Plugin):
    """This plugin provides test results for the Jenkins PerfPerformance plugin."""
    name = 'jenkinsperf'
    score = 499 # necessary for it to go after capture
    encoding = 'UTF-8'
    xunitstream = None
    xunitstats = None
    xunit_file = None
    resulttags = None
    platform = None

    def _quoteattr(self, attr):
        """Escape an XML attribute. Value can be unicode."""
        attr = xml_safe(attr)
        if isinstance(attr, unicode) and not UNICODE_STRINGS:
            attr = attr.encode(self.encoding)
        return saxutils.quoteattr(attr)

    def options(self, parser, env):
        """Sets additional command line options."""
        Plugin.options(self, parser, env)
        parser.add_option(
            '--jenkinsperf-file', action='store',
            dest='jenkinsperf_file', metavar="FILE",
            default=env.get('NOSE_JENKINSPERF_FILE', 'nosetests.xml'),
            help=("Path to xml file to store the jenkins perf report in. "
                  "Default is nosetests.xml in the working directory "
                  "[NOSE_JENKINSPERF_FILE]"))
        parser.add_option(
            '--jenkinsperf-header', action='store',
            dest='jenkinsperf_header', metavar="HEADER",
            default=env.get('NOSE_JENKINSPERF_HEADER', 'name="nosetests" categ="nosetests"'),
            help=("The attributes of the <testsuite> report that will be created, in particular 'package' and 'name' should be filled."
                  "[NOSE_JENKINSPERF_HEADER]"))

    def configure(self, options, config):
        Plugin.configure(self, options, config)
        self.config = config
        if self.enabled:
            self.xunitstream = globaljenkinsstream
            self.xunitstats = globalxunitstats
            for i in range(4):
                self.xunitstats[i] = 0
            self.xunit_file = options.jenkinsperf_file
            self.jenkinsperf_header = options.jenkinsperf_header
            self.resulttags = ['compiletime', 'performance', 'executiontime','targets']
            self.platform = '<platform><os><type><![CDATA[%s]]></type><name><![CDATA[%s]]></name></os><processor arch="%s"></processor><compiler>%s</compiler><environment></environment></platform>'%(platform.platform(),platform.system(), platform.machine(),platform.python_compiler())

    def report(self, stream):
        """Writes an Jenkins Perf-formatted XML file

        The file includes a report of test errors and failures.

        """
        #stats = {'errors': self.xunitstats[0], 'failures': self.xunitstats[1], 'passes': self.xunitstats[2], 'skipped': self.xunitstats[3] }
        #stats['encoding'] = self.encoding
        #stats['total'] = (stats['errors'] + stats['failures'] + stats['passes'] + stats['skipped'])
        #stats['header'] = self.jenkinsperf_header
        if UNICODE_STRINGS:
            error_report_file = open(self.xunit_file, 'w', encoding=self.encoding)
        else:
            error_report_file = open(self.xunit_file, 'w')
        error_report_file.write(
            '<?xml version="1.0" encoding="%s"?>\n'
            '<report %s><start></start>'%(self.encoding,self.jenkinsperf_header))
        while len(self.xunitstream) > 0:
            error_report_file.write(self.xunitstream.pop(0))
        #error_report_file.write('<properties><property name="myproperty" value="1.5"/></properties>')
        error_report_file.write('</report>')
        error_report_file.close()
        if self.config.verbosity > 1:
            stream.writeln("-" * 70)
            stream.writeln("XML: %s" % error_report_file.name)

    def startTest(self, test):
        """Initializes a timer before starting a test."""
        pass

    def extractResults(self, output):
        result = ''
        targets = ''
        for tag in self.resulttags:
            while True:
                startindex = output.find('<'+tag)
                if startindex < 0:
                    break
                endindex = output.find('/>',startindex+len(tag)+2)
                if endindex < 0:
                    endindex = output.find('/'+tag+'>',startindex+len(tag)+2)
                    if endindex < 0:
                        break
                    else:
                        endindex += 2+len(tag)
                else:
                    endindex += 2
                if tag == 'targets':
                    targets += output[startindex:endindex]
                else:
                    result += output[startindex:endindex]
                output = output[:startindex] + output[endindex:]
        return output, result, targets
        
    def addError(self, test, err, capt=None):
        executed = 'yes'
        if issubclass(err[0], SkipTest):
            executed = 'no'
            self.xunitstats[3] += 1
        else:
            self.xunitstats[0] += 1
        tb = ''.join(traceback.format_exception(*err))
        id=test.shortDescription()
        if id is None:
            id = test.id()
        systemout = ''
        result = ''
        targets = ''
        if test.capturedOutput is not None:
            # look for a result tag in the output
            output, result, targets = self.extractResults(str(test.capturedOutput))
            systemout = '<log name="stdout"><![CDATA['+escape_cdata(output)+']]></log>'            
        xml = """<test name="%s" executed="%s">
%s
%s
"""%(id, executed, self.platform, targets)

        if executed == 'yes':
            timedout = 'true' if nice_classname(err[0]).find('TimedOutException') >= 0 else 'false'
            xml += """
<result>
<success passed="no" state="1" hasTimedOut="%s"/>
%s
%s
<errorlog>%s</errorlog>
<log name="traceback">%s</log>
</result>
"""%(timedout, result, systemout, self._quoteattr(exc_message(err)),escape_cdata(tb))
            #'errtype': self._quoteattr(nice_classname(err[0]))
        xml += '\n</test>\n'
        self.xunitstream.append(xml)

    def addFailure(self, test, err, capt=None, tb_info=None):
        tb = ''.join(traceback.format_exception(*err))
        self.xunitstats[1] += 1
        id=test.shortDescription()
        if id is None:
            id = test.id()
        systemout = ''
        result = ''
        targets = ''
        if test.capturedOutput is not None:
            # look for a result tag in the output
            output, result, targets = self.extractResults(str(test.capturedOutput))
            systemout = '<log name="stdout"><![CDATA['+escape_cdata(output)+']]></log>'            
        xml = """<test name="%s" executed="yes">
%s
%s
"""%(id, self.platform, targets)

        xml += """
<result>
<success passed="no" state="0" hasTimedOut="false"/>
%s
%s
<errorlog>%s</errorlog>
<log name="traceback">%s</log>
</result>
"""%(result, systemout, self._quoteattr(exc_message(err)),escape_cdata(tb))
        xml += '\n</test>\n'
        self.xunitstream.append(xml)

    def addSuccess(self, test, capt=None):
        print 'yooooooooooooooo'
        self.xunitstats[2] += 1
        id=test.shortDescription()
        if id is None:
            id = test.id()
        systemout = ''
        result = ''
        targets = ''
        if test.capturedOutput is not None:
            # look for a result tag in the output
            output, result, targets = self.extractResults(str(test.capturedOutput))
            systemout = '<log name="stdout"><![CDATA['+escape_cdata(output)+']]></log>'    
        xml = """<test name="%s" executed="yes">
%s
%s
"""%(id, self.platform, targets)

        xml += """
<result>
<success passed="yes" state="100" hasTimedOut="false"/>
%s
%s
</result>
"""%(result, systemout)
        xml += '\n</test>\n'
        self.xunitstream.append(xml)
