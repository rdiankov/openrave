#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import re
from optparse import OptionParser

if __name__ == "__main__":
    parser = OptionParser(description='Preprocesses and build the latex genereted doxygen scripts',
                          usage='%prog [options] latex_directory')

    (options, args) = parser.parse_args()
    
    # preprocess the index
    refman_file = os.path.join(args[0],'refman.tex')
    doctext = open(refman_file,'r').read()
    lines = doctext.splitlines()
#     filedocindex = [i for i,s in enumerate(lines) if s.find('\\section{File Documentation}') >= 0][0]
#     if filedocindex >= 0:
#         lines.pop(filedocindex)
#         while filedocindex < len(lines):
#             if lines[filedocindex].find('\\input{') < 0:
#                 break
#             lines.pop(filedocindex)
    doctext = '\n'.join(lines)
    doctext = re.sub('\\\\section\{','\\clearemptydoublepage\n\\section{',doctext)
    open(refman_file,'w').write(doctext)

    os.chdir(args[0])
    # yes, 3 times
    os.system('pdflatex refman.tex')
    os.system('pdflatex refman.tex')
    os.system('pdflatex refman.tex')
