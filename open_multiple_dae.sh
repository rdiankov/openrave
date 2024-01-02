#!/bin/bash
# This constructs the Python command to load all selected .dae files
python_command="for file in ["
for file in "$@"; do
    python_command+="'"$file"', "
done
python_command=${python_command%, }  # Removes the trailing comma
python_command+="]: env.Load(file)"

# Now call openrave.py with the constructed Python command
openrave.py --viewer qtcoin -i -p "$python_command"