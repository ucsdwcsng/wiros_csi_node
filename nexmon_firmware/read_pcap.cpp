//compile with c++17

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <utility>
#include <bitset>
#include <map>
#include <array>
#include <iterator>
#include <algorithm>
#include <unordered_map>
#include <string.h>

typedef unsigned char u8;
typedef std::array<uint8_t, 6> mac;
#define k_tof_unpack_sgn_mask (1<<31)
#define H_OFFSET 64
#define TIMESTAMP_BYTE_OFFSET 42

uint32_t version_number = 1;

//two bytes to uint16_t
uint16_t chtoush(char u, char l)
{
	return (((uint16_t)u) << 8) | (0x00ff & l);
};

uint32_t chtouint(uint8_t in[4])
{
	return ((uint32_t)in[0] << 24) |  ((uint32_t)in[1] << 16) |  ((uint32_t)in[2] << 8) | (uint32_t)in[3];  
}

//contains the information from one packet. Has relevent metadata and the CSI for one 1x1 spatial stream.
class csi_instance
{
public:
	uint32_t time_sec = 0;
	uint32_t time_usec;
	unsigned char source_addr[4];
	unsigned char dest_addr[4];

	uint16_t source_port;
	uint16_t dest_port;
	uint16_t body_len;
	unsigned char source_mac[6];
	uint16_t seq_num;
	uint8_t rssi;
	unsigned char tx;
	unsigned char rx;
	unsigned char channel;
	unsigned char bw;
	size_t csi_size;
	int32_t* csi;
};

//human readable mac address
std::string hr_mac(unsigned char* source_mac)
{
	char source_mac_str[19]; 
	sprintf(source_mac_str, "%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx:%.2hhx", source_mac[0], source_mac[1], source_mac[2], source_mac[3], source_mac[4], source_mac[5]);
	return std::string(source_mac_str);
}

int txrx_ptr(unsigned char tx, unsigned char rx)
{
	return rx + 4*tx;
}

//class that takes csi_instances and compiles them to give comlete NxSS matrices.
void write_csi(std::ostream &out, std::vector<csi_instance> data)
{
	const int32_t n_rx = 4;
	const int32_t n_tx = 4;

	//setup some metadata as well as time and rssi
	csi_instance first = data[0];
	//this will be n_subcarriers * 2 (since we have a real and imaginary part)
	uint32_t band_size = first.csi_size;
	uint32_t time_sec = first.time_sec;
	uint32_t time_usec = first.time_usec;
	uint32_t rssi = (uint32_t)first.rssi;
	uint32_t channel = (uint32_t)first.channel;
	// std::cout << band_size << std::endl;

	//allocate a buffer to fill in our n_subx4x4 CSI matrix
	uint32_t csi_mat_size = band_size * n_rx * n_tx;
	int32_t csi_out[csi_mat_size] = {};

	//write mac address
	out.write(reinterpret_cast<const char*>(first.source_mac), 6);
	out.write(reinterpret_cast<const char*>(&csi_mat_size), 4);
	out.write(reinterpret_cast<const char*>(&n_rx), 4);
	out.write(reinterpret_cast<const char*>(&n_tx), 4);
	out.write(reinterpret_cast<const char*>(&band_size), 4);
	out.write(reinterpret_cast<const char*>(&time_sec), 4);
	out.write(reinterpret_cast<const char*>(&time_usec), 4);
	out.write(reinterpret_cast<const char*>(&rssi), 4);
	out.write(reinterpret_cast<const char*>(&channel), 4);

	for(int nss = 0; nss < data.size(); ++nss)
	{
		//offset (in int32_t) of our current tx and rx data in the total CSI matrix
		csi_instance csi_in = data[nss];
		size_t offset = txrx_ptr(csi_in.tx, csi_in.rx) * band_size;
		for(int i = 0; i < band_size; ++i)
			csi_out[offset + i] = csi_in.csi[i];
	}
	out.write(reinterpret_cast<const char*>(&csi_out), csi_mat_size * sizeof(int32_t));
	// std::cout << reinterpret_cast<const char*>(&csi_out), csi_mat_size * sizeof(int32_t));
}

void unpack_float_acphy(int nbits, bool autoscale, int shft, 
	int fmt, int nman, int nexp, int nfft, 
	uint32_t *H, int32_t *Hout)
{
	int e_p, maxbit, e, i, pwr_shft = 0, e_zero, sgn;
	int n_out, e_shift;
	int8_t He[256];
	int32_t vi, vq, *pOut;
	uint32_t x, iq_mask, e_mask, sgnr_mask, sgni_mask;
	//real_imaginary mask
	iq_mask = (1<<(nman-1))- 1;
	//exponent bitmask
	e_mask = (1<<nexp)-1;
	//
	e_p = (1<<(nexp-1));
	//sign of real part bitmask
	sgnr_mask = (1 << (nexp + 2*nman - 1));
	sgni_mask = (sgnr_mask >> nman);
	e_zero = -nman;
	pOut = (int32_t*)Hout;
	n_out = (nfft << 1);
	e_shift = 1;
	maxbit = -e_p;
	for (i = 0; i < nfft; i++) {
		//read imaginary part
		vi = (int32_t)((H[i] >> (nexp + nman)) & iq_mask);
		//read real part
		vq = (int32_t)((H[i] >> nexp) & iq_mask);
		//read exponent
		e =   (int)(H[i] & e_mask);
		if (e >= e_p)
			e -= (e_p << 1);
		He[i] = (int8_t)e;
		x = (uint32_t)vi | (uint32_t)vq;
		if (autoscale && x) {
			uint32_t m = 0xffff0000, b = 0xffff;
			int s = 16;
			while (s > 0) {
				if (x & m) {
					e += s;
					x >>= s;
				}
				s >>= 1;
				m = (m >> s) & b;
				b >>= s;
			}
			if (e > maxbit)
				maxbit = e;
		}
		if (H[i] & sgnr_mask)
			vi |= k_tof_unpack_sgn_mask;
		if (H[i] & sgni_mask)
			vq |= k_tof_unpack_sgn_mask;
		Hout[i<<1] = vi;
		Hout[(i<<1)+1] = vq;
	}
    //shft = nbits - maxbit;
	if (shft == 32)
		shft = nbits - maxbit;
	for (i = 0; i < n_out; i++) {
		e = He[(i >> e_shift)] + shft;
		vi = *pOut;
		sgn = 1;
		if (vi & k_tof_unpack_sgn_mask) {
			sgn = -1;
			vi &= ~k_tof_unpack_sgn_mask;
		}
		if (e < e_zero) {
			vi = 0;
		} else if (e < 0) {
			e = -e;
			vi = (vi >> e);
		} else {
			vi = (vi << e);
		}
		*pOut++ = (int32_t)sgn*vi;
	}
}

int main(int argc, char*argv[])
{
	// process args
	std::string trace_file;
	std::vector<std::string> args(argv, argv + argc);
	if(std::find(begin(args), end(args), std::string("-h")) != end(args))
	{
		std::cout << "USAGE:\n"
		"readpcap [input filename] (-o [output filename]) (-n)\n"
		"-h: print this help menu\n"
		"-o: name of output file, will default to [input file].csi if not provided\n"
		"-s: value to shift by, don't set or set to 32 to use autoscale\n"
		"-n: don't autoscale the magnitudes.\n" << std::endl;
		exit(1);
	}
	if(args.size() > 1)
	{
		trace_file = args[1];
	}
	else
	{
		std::cout << "No input file!" << std::endl;
		exit(0);
	}
	std::ifstream f;
	f.open(trace_file);
	if(f.fail())
	{
		std::cout << "Bad input file!" << std::endl;
		exit(0);
	}
	//associate either cout or our output file with one stream
	std::streambuf *ofbuf;
	std::ofstream of;
	auto out_arg = std::find(begin(args), end(args), std::string("-o"));
	if(out_arg != args.end())
	{
		of.open(std::string(*(++out_arg)));
		ofbuf = of.rdbuf();
	}
	else
	{
		ofbuf = std::cout.rdbuf();
	}

	std::ostream out(ofbuf);

	out.write(reinterpret_cast<const char*>(&version_number), 4);


	int32_t shift;
	auto shift_arg = std::find(begin(args), end(args), std::string("-s"));
	if(shift_arg != args.end())
	{
		shift = std::stoi(std::string(*(++shift_arg)));
	}
	else
	{
		shift = 32;
	}

	

	std::stringstream buf;
	buf << f.rdbuf();
	std::string bufstr(buf.str());

	//look for beginnings of a CSI UDP packet
	size_t pkt_it = 0;
	std::vector<size_t> packets;
	while(pkt_it < bufstr.size())
	{
		pkt_it = bufstr.find("\x0a\x0a\x0a\x0a\xff\xff\xff\xff", pkt_it + 1);
		packets.push_back(pkt_it);
	}
	std::cout << packets.size() << " Packets Detected" << std::endl;

	uint16_t old_seq_num = 0;
	int seq_count = 1;

	char old_mac[6];

	std::vector<csi_instance> csi_data;

	for(auto pkt : packets)
	{
		seq_count++;
		//extract source and destination IPs and Ports
		csi_instance data;
		//look before the udp packet starts to get pcap timestamp for this frame


		buf.seekg(pkt - TIMESTAMP_BYTE_OFFSET);
		char time_sec[4];
		buf.read(time_sec, 4);
		char time_usec[4];
		buf.read(time_usec, 4);
		

		uint32_t* timestamp = reinterpret_cast<uint32_t*>(time_sec);
		uint32_t* utime = reinterpret_cast<uint32_t*>(time_usec);
		
		data.time_sec = *timestamp;
		data.time_usec = *utime;

		buf.seekg(pkt);
		char source_addr[4];
		buf.read(source_addr, 4);

		for(int i = 0; i < 4; i++)
			data.source_addr[i] = source_addr[i];

		char dest_addr[4];
		char dest_addr_str[100];
		buf.read(dest_addr, 4);
		sprintf(dest_addr_str, "%.hhu.%.hhu.%.hhu.%.hhu", dest_addr[0], dest_addr[1], dest_addr[2], dest_addr[3]);
		memcpy(data.dest_addr, dest_addr, 4);
		// for(int i = 0; i < 4; i++)
		// 	data.dest_addr[i] = dest_addr[i];
		char source_port_bytes[2];
		buf.read(source_port_bytes, 2);
		uint16_t source_port = chtoush(source_port_bytes[0], source_port_bytes[1]);
		data.source_port = source_port;

		char dest_port_bytes[2];
		buf.read(dest_port_bytes, 2);
		uint16_t dest_port = chtoush(dest_port_bytes[0], dest_port_bytes[1]);
		data.dest_port = dest_port;

		char body_len_bytes[4];
		buf.read(body_len_bytes, 4);
		uint16_t body_len = chtoush(body_len_bytes[0], body_len_bytes[1]) - 8;
		data.body_len = body_len;

		char magic_number[2];
		buf.read(magic_number, 2);


		char rssi[1];
		buf.read(rssi, 1);
		data.rssi = rssi[0];

		char fc[1];
		buf.read(fc, 1);

		char source_mac[6];
		buf.read(source_mac, 6);
		memcpy(data.source_mac, source_mac, 6);

		char seq_num_bytes[2];
		buf.read(seq_num_bytes, 2);
		uint16_t seq_num = chtoush(seq_num_bytes[0], seq_num_bytes[1]);
		data.seq_num = seq_num;

		char nss_idx[2];
		buf.read(nss_idx, 2);

		//bitmask for rx and tx ant
		char tx_ant = (nss_idx[1] >> 3) & 0x3;
		char rx_ant = nss_idx[1] & 0x3;
		data.tx = tx_ant;
		data.rx = rx_ant;

		char chanspec[2];
		buf.read(chanspec, 2);
		char cs_str[20];
		// sprintf(cs_str, "%X%X", (uint8_t)chanspec[0], (uint8_t)chanspec[1]);
		// std::cout << cs_str << std::endl;
		// std::cout << "bad bw detected:" << cs_str << " " << (int)bw << std::endl;
		char channel = chanspec[0];
		data.channel = channel;
		char sideband = (chanspec[1] >> 6) & 0x03;
		char bw = (chanspec[1]>>3) & 0x07;
		// char bw_str[20];
		// sprintf(bw_str, "%X", (uint8_t)bw);
		// std::cout << bw_str << std::endl;
		// char band = 2;

		switch(bw)
		{
			case 0x4:
			bw = 80;
			break;
			case 0x3:
			bw = 40;
			break;
			case 0x2:
			bw = 20;
			break;
			default:
			break;
		}
		data.bw = bw;
		//char chanstr[20];
		//sprintf(chanstr, "channel: %.hhu ", channel);
		//char bwstr[20];
		//sprintf(bwstr, "bw: %.hhuMHz ", bw);
		//char bandstr[20];
		//sprintf(bandstr, "band: %.hhu ", band);
		//char chanspecstr[20];
		//sprintf(chanspecstr, "spec: 0x%hhX%hhX", chanspec[0], chanspec[1]);

		//std::cout << chanstr << bwstr << bandstr << std::endl;
		//std::cout << chanspecstr << std::endl;
		char chip_data[2];
		buf.read(chip_data, 2);

		size_t csi_nbytes;
		//3.2 subcarriers per MHz bandwidth, four bytes of CSI per subcarreir
		uint32_t n_sub = (uint32_t)((float)bw * 3.2);
		csi_nbytes = (size_t)(n_sub * 4);
		buf.seekg(pkt + 34);
		char csi[csi_nbytes];
		buf.read(csi, csi_nbytes);

		uint32_t csi_ints[n_sub * 2];

		int byte_order [4] = {0,1,2,3};
		for(int i = 0; i < n_sub; i++)
		{
			//std::cout << i << std::endl;

			//first one should be 12 4a a7 34 == 306882356
			int k = 4*i;
			uint32_t csi_int = ((u8)csi[k + byte_order[0]]) | ((u8)csi[k + byte_order[1]] << 8) | ((u8)csi[k + byte_order[2]] << 16) | ((u8)csi[k + byte_order[3]] << 24);
			csi_ints[i] = csi_int;
		}
		int32_t cmplx_out_shift[n_sub * 2];
		for(int i = 0; i < n_sub*2; ++i){
			cmplx_out_shift[i] = 0;
		}
		//int shift = argv[2] ? std::stoi(argv[2]) : 32;
		bool is_shifted=false;
		int shft = shift;
		while(!is_shifted){
			for(int i = 0; i < n_sub*2; ++i){
				if(cmplx_out_shift[i] > 1000){
					is_shifted=true;
					break;
				}
			}
			unpack_float_acphy(10, 1, shft, 1, 12, 6, n_sub, csi_ints, cmplx_out_shift);
			shft += 1;
		}
		
		int32_t cmplx_out[n_sub *2];
		for(int i = n_sub; i < 2*n_sub; i++)
		{
			cmplx_out[i - n_sub] = cmplx_out_shift[i];
		}
		for(int i = 0; i < n_sub; i++)
		{
			cmplx_out[i + n_sub] = cmplx_out_shift[i];
		}
		/*for(int i = 0; i < n_sub; i++)
		{
			std::bitset<32> b_out(csi_ints[i]);
			std::cout << b_out << " " << csi_ints[i] << " >> " << cmplx_out[2*i] << " " << cmplx_out[2*i + 1] << std::endl;
		}*/
		data.csi = new int32_t[n_sub*2];
		data.csi_size = n_sub*2;

		for(int i = 0; i < n_sub*2; i++)
		{
			data.csi[i] = cmplx_out[i];
		}

		//if this is a new transmission, write out the data we have found
		if(data.tx == 0 && data.rx == 0 && csi_data.size() > 0)
		{
			write_csi(of, csi_data);
			while(!csi_data.empty())
				csi_data.pop_back();
		}

		csi_data.push_back(data);
	}
	f.close();
}