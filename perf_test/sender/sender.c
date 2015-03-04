#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ieee802154.h>

int pkt_size = 115;

int main(int argc, char **argv) {
	int sd;
	int ret;
	struct sockaddr_ieee802154 a;

	if (argv[1] && !strcmp(argv[1], "--version")) {
		printf(	"sender\n");
		return 0;
	}

	if (!argv[1] || !strcmp(argv[1], "--help") || argc != 4) {
		printf("Usage: %s PANid sourceAddr destAddr\n", argv[0]);
		printf("  or:  %s --help\n", argv[0]);
		return 0;
	}

	sd = socket(PF_IEEE802154, SOCK_DGRAM, 0);
	if (sd < 0) {
		perror("socket");
		return 1;
	}

	a.family = AF_IEEE802154;
	a.addr.addr_type = IEEE802154_ADDR_SHORT;

	a.addr.pan_id = strtol(argv[1], NULL, 16);

	a.addr.short_addr = (strtol(argv[2], NULL, 16));
	ret = bind(sd, (struct sockaddr *)&a, sizeof(a));
	if (ret) {
		perror("bind");
		return 1;
	}

	a.addr.short_addr = (strtol(argv[3], NULL, 16));
	ret = connect(sd, (struct sockaddr *)&a, sizeof(a));
	if (ret) {
		perror("connect");
		return 1;
	}

	while (1) {
		int count;
		int bad,good;
		double time,speed;
		struct timespec start, stop;

		ret = clock_gettime(CLOCK_MONOTONIC, &start);
		if (ret) {
			perror("clock_gettime");
		}

		bad = 0, good = 0;

		for(count=0;count<100;count++) {
			fd_set set;
			struct timeval timeout;
			char buf[127];

			write(sd, &buf, pkt_size);

			FD_ZERO(&set);
			FD_SET(sd, &set);

			timeout.tv_sec = 0;
			timeout.tv_usec = 500000;

			ret = select(sd + 1, &set, NULL, NULL, &timeout);
			switch(ret) {
				case -1:
					perror("select");
					break;
				case 0:
					bad++; // write(1, "t", 1);
					break;
				default:
					ret = read(sd, &buf, sizeof(buf));
					if(ret == pkt_size)
						good++; // write(1, ".", 1);
					else
						bad++; // write(1, "x", 1);
			}
		}

		ret = clock_gettime(CLOCK_MONOTONIC, &stop);
		if (ret) {
			perror("clock_gettime");
		}

		time = (1.0 * stop.tv_sec + 1.0 * stop.tv_nsec/1000000000.0)
			- (1.0 * start.tv_sec + 1.0 * start.tv_nsec/1000000000.0);
		speed = 2.0 * pkt_size * good / time;

		fprintf(stdout,
			"%d packets (%d bad, %d good) with %d size for %3.3f sec (%4.3f bytes/sec)\n",
			count, bad, good, pkt_size, time, speed);

	}
	return 0;
}
