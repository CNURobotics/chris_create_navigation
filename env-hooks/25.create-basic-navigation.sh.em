# Set some sane defaults for the CHRIS navigation launch environment

##Documentation:
#  The colon command simply has its arguments evaluated and then succeeds.
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character.
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside.
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${CHRIS_CREATE_MAP_FILE:=`rospack find chris_world_models`/maps/empty.yaml}

# Exports
export CHRIS_CREATE_MAP_FILE
