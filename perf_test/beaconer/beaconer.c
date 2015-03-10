#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ieee802154.h>


#define SHORT_SIZE	15
#define MEDIUM_SIZE	63
#define LARGE_SIZE	115

int sd;

char in_buffer[LARGE_SIZE];
long in_counter = 0;

char out_short[SHORT_SIZE];
char out_medium[MEDIUM_SIZE];
char out_large[LARGE_SIZE];
long out_counter = 0;

void* send_beacon_pthread( void* arg )
{
	int ret;
	while(1) {
		sprintf(out_short,"s[%10ld]\n",out_counter);
		ret = write(sd,&out_short,SHORT_SIZE);
		if (ret < 0) {
			perror("write");
			pthread_exit(NULL);
		}
		fprintf(stdout,"S> %s", out_short);
		sleep(1);

		sprintf(out_medium,"m[%10ld]\n",out_counter);
		ret = write(sd,&out_medium,MEDIUM_SIZE);
		if (ret < 0) {
			perror("write");
			pthread_exit(NULL);
		}
		fprintf(stdout,"M> %s", out_medium);
		sleep(1);

		sprintf(out_large,"l[%10ld]\n",out_counter);
		ret = write(sd,&out_large,LARGE_SIZE);
		if (ret < 0) {
			perror("write");
			pthread_exit(NULL);
		}
		fprintf(stdout,"L> %s", out_large);
		sleep(1);

		out_counter++;
	}
}

int main(int argc, char **argv) {
	pthread_t pth_send;
	int ret;
	struct sockaddr_ieee802154 a;


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

	ret = pthread_create(&pth_send, NULL, send_beacon_pthread, NULL);
	if(ret) {
		perror("pthread_create");
		exit(-1);
	}

	while (1) {
		ret = read(sd,&in_buffer,LARGE_SIZE);
		if (ret < 0) {
			perror("read");
			exit(-1);
		}
		switch(ret){
			case LARGE_SIZE:
				fprintf(stdout,"L< %s",in_buffer);
				break;
			case MEDIUM_SIZE:
				fprintf(stdout,"M< %s",in_buffer);
				break;
			case SHORT_SIZE:
				fprintf(stdout,"S< %s",in_buffer);
				break;
			default:
				fprintf(stdout,"?< %s",in_buffer);
				break;
		}

	}
	return 0;
}
