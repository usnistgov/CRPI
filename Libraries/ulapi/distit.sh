#!/bin/sh

cd /tmp && rm -rf ulapi && git clone git@github.com:usnistgov/el-robotics-core.git && cd el-robotics-core/ulapi && ./autoconf.sh && make dist && tar xzvf *.tar.gz && cd ulapi-* && ./configure && make && make dist || exit 1

exit 0

