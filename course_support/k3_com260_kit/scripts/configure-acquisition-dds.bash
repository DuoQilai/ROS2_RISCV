#!/usr/bin/env bash
set -eu
if [[ $# != 2 || ! $1 =~ ^[0-9a-fA-F:]+:[0-9a-fA-F:]*$ || ! $2 =~ ^[0-9a-fA-F:]+:[0-9a-fA-F:]*$ || $1 == "$2" ]]; then
  echo 'Usage: configure-acquisition-dds.bash LOCAL_IPV6 PEER_IPV6' >&2
  exit 1
fi
directory="$HOME/.config/ros2-course-com260-ch22-ch23"
mkdir -p "$directory"
cat > "$directory/dds.bash" <<ENV
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0 ROS_DOMAIN_ID=222
export CYCLONEDDS_URI='<CycloneDDS><Domain>
<General><Transport>udp6</Transport><Interfaces><NetworkInterface address="$1"/></Interfaces>
<AllowMulticast>false</AllowMulticast><MaxMessageSize>1400B</MaxMessageSize><FragmentSize>1280B</FragmentSize></General>
<Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
<Peers><Peer Address="[::1]"/><Peer Address="[$2]"/></Peers></Discovery></Domain></CycloneDDS>'
ENV
