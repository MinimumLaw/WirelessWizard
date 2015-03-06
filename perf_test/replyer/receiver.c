#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ieee802154.h>

int main(int argc, char **argv) {
	int sd;
	int ret;
	int count = 0;
	struct sockaddr_ieee802154 a;
	size_t recv = 0;


	if (argv[1] && !strcmp(argv[1], "--version")) {
		printf(	"receiver \n");
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
		char buf[127];

		ret = read(sd,&buf,sizeof(buf));
		if (ret < 0) {
			perror("read");
			exit(-1);
		}
		recv += ret;
		if(recv >= (1024*1024)) { /* every 1KiB show dot */
			write(1, ".", 1);
			recv -= (1024*1024);
		}

	}
	return 0;
}
