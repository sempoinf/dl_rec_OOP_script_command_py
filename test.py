import sys


def main(args: list):
    snss = [3, 46]
    ranges = [1, 2]
    sample_size: int=10
    data_buff_sns_r = {sns: {rnge: [None] * sample_size for rnge in ranges} for sns in snss}
    index = 0
    for frame_num in range(sample_size):
        for sns_index, sns in enumerate(snss):
            data_buff_r = {}
            for rnge_index, rnge in enumerate(ranges):
                data_buff_r[rnge] = index
                index += 1
            print(data_buff_r)
            for rnge, value in data_buff_r.items():
                data_buff_sns_r[sns][rnge][frame_num] = value
    
    print(f"\n{data_buff_sns_r}")

if __name__ == '__main__':
    main(sys.argv)