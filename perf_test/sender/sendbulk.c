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


char BulkData[1024*1024]; /* 1 MiB data */

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

//	while (1) {
		int send_remain = sizeof(BulkData);
		char *send_ptr = BulkData;
		int send_now = 0;
		int count = 0;

		double time,speed;
		struct timespec start, stop;

		ret = clock_gettime(CLOCK_MONOTONIC, &start);
		if (ret) {
			perror("clock_gettime");
		}

		while(send_remain != 0) {
			send_now = write(sd, send_ptr, 115 < send_remain ? 115 : send_remain);
			if(send_now == -1) {
				perror("write");
				exit(-1);
			}
			send_ptr += send_now;
			send_remain -= send_now;
			count++;
			fprintf(stdout,"PKT %d (size %d)\n",
				count, send_now);
		};
		ret = clock_gettime(CLOCK_MONOTONIC, &stop);
		if (ret) {
			perror("clock_gettime");
		}

		time = (1.0 * stop.tv_sec + 1.0 * stop.tv_nsec/1000000000.0)
			- (1.0 * start.tv_sec + 1.0 * start.tv_nsec/1000000000.0);
		speed = 1.0 * sizeof(BulkData) / time;

		fprintf(stdout,
			"%ld bytes (%d packets) for %3.3f sec (%4.3f bytes/sec)\n",
			sizeof(BulkData), count, time, speed);

//	}
	return 0;
}
