#! /bin/bash
#name--create_cscope.sh
find . -name "*.h" -o -name "*.c" -o -name "*.cpp" -o -name "*.cc" -o -name "*.hh" > cscope.file
cscope -Rbq -i cscope.file
ctags -R --c++-kinds=+px --fields=+iaS --extra=+q -L cscope.file

rm -rf cscope.file
echo create tags file success
echo create cscope file success
