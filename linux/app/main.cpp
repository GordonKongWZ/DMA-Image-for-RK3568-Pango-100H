#include "../includes/pcie_dma_read_test.h"
#include <iostream>
#include <chrono>
#include <cstdint>
#include <cstddef>
#include <opencv2/opencv.hpp>
using namespace std;



//=========================================================================
// *作   者：可乐大盗
// *完成时间：2025年7月21日
// *实现功能：简单的1080p图像采集测试
// *备   注：
//=========================================================================
#define IMAGE_WIDTH  1920
#define IMAGE_HEIGHT 1080


//=========================================================================
// 函数声明
//=========================================================================
int open_pci_driver(void);
void cmd_operation(int fd, unsigned char cmd, COMMAND_OPERATION *cmd_op);
int nano_delay(long delay);
void rgb565_to_rgb888(const uint16_t* image565, uint8_t* image888, size_t num_pixels);


//=========================================================================
// 全局变量定义
//=========================================================================
int pci_driver_fd;
COMMAND_OPERATION command_operation;

DMA_OPERATION dma_operation;  //读写之前需操作这个结构体设置好对应的参数
//char image_buf_temp[1920*1080*2];   //image读数据暂存
uint8_t* image_buf_temp = (uint8_t*)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 2);
//char image_buf_888[1920*1080*3];
uint8_t* image_buf_888 = (uint8_t*)malloc(IMAGE_WIDTH * IMAGE_HEIGHT * 3);
/**************************************************************************
** 函数名称:    open_pci_driver
** 函数功能:    打开pcie驱动文件
** 输入参数:    无
** 输出参数:    无
** 返回参数:    文件描述符
****************************************************************************/
int open_pci_driver(void)
{
	int fd;
	
	fd = open(PCIE_DRIVER_FILE_PATH, O_RDWR);
	
	if(fd < 0)
	{
		perror("open fail\n");
		return -1;
	}
	return fd;
}

/**************************************************************************
** 函数名称:    cmd_operation
** 函数功能:    命令解析函数
** 输入参数:    fd:驱动文件描述符
**			cmd:操作指令
**			*cmd_op:相关信息和数据结构体
** 输出参数:    无
** 返回参数:    无
****************************************************************************/
void cmd_operation(int fd, unsigned char cmd, COMMAND_OPERATION *cmd_op)
{
	unsigned int w_data = 0;
	unsigned int r_data = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int cnt = 0;
	unsigned long temp_bar_len;
	int value = 0;
	char pci_info[20][20];
	char unit[5][10] = {"B", "KB", "MB", "GB", "TB"};
	void *vaddr1, *vaddr2;
	switch(cmd)
	{
		case info_num:
			cmd_op->delay = 0;
			read(fd, cmd_op, sizeof(COMMAND_OPERATION));								/* 读取pcie信息 */
			sprintf(pci_info[cnt++], "%04x", cmd_op->get_pci_dev_info.vendor_id);
			sprintf(pci_info[cnt++], "%04x", cmd_op->get_pci_dev_info.device_id);
			if(cmd_op->cap_info.cap_status == 1)
			{
				strcpy(pci_info[cnt++],  "Up");
				printf("PCIe link successful\n");
				printf("PCIe link successful\n");
			}
			else
			{
				strcpy(pci_info[cnt++],  "Down");
				printf("PCIe link failure !!!\n");
				printf("PCIe link failure !!!\n");
			}
			sprintf(pci_info[cnt++], "Gen%x", cmd_op->get_pci_dev_info.link_speed);
			sprintf(pci_info[cnt++], "x%x", cmd_op->get_pci_dev_info.link_width);
			strcpy(pci_info[cnt++],  "No");
			for(i = 0; i <= 5; i++)
			{
				sprintf(pci_info[cnt++], "%08lx", cmd_op->get_pci_dev_info.bar[i].bar_base);
			}
			for(i = 0; i <= 5; i++)
			{
				value = 0;
				temp_bar_len = cmd_op->get_pci_dev_info.bar[i].bar_len;
				while(temp_bar_len >= 1024)
				{
					temp_bar_len = temp_bar_len / 1024;
					value++;
				}
				printf(pci_info[cnt++], "%d%s", temp_bar_len, unit[value]);
			}
			printf(pci_info[cnt++], "%d", cmd_op->get_pci_dev_info.mps);
			printf(pci_info[cnt++], "%d", cmd_op->get_pci_dev_info.mrrs);
			
		break;

		
		case performance_num:
		
		break;
		default:
		break;
	}

}
/**************************************************************************
** 函数名称:    nano_delay
** 函数功能:纳秒延时
** 输入参数:delay：延时长度
** 输出参数:    无
** 返回参数:    无
****************************************************************************/
int nano_delay(long delay)
{
    struct timespec req, rem;
    long nano_delay = delay;
    int ret = 0;
    while(nano_delay > 0)
    {
        rem.tv_sec = 0;
        rem.tv_nsec = 0;
        req.tv_sec = 0;
        req.tv_nsec = nano_delay;
        if(ret = (nanosleep(&req, &rem) == -1))
        {
            printf("nanosleep failed !!!\n");                
        }
        nano_delay = rem.tv_nsec;
    };
    
    return ret;
}


/**************************************************************************
** 函数名称:rgb565_to_rgb888
** 函数功能:将rgb565图像转为rgb888图像
** 输入参数:
** 输出参数:    无
** 返回参数:    无
****************************************************************************/
void rgb565_to_rgb888(const uint16_t* image565, uint8_t* image888, size_t num_pixels) {
    for (size_t i = 0; i < num_pixels; ++i) {
        uint16_t pixel = image565[i];
        uint8_t r = ((pixel >> 11) & 0x1F) << 3;
        uint8_t g = ((pixel >> 5 ) & 0x3F) << 2;
        uint8_t b = (pixel & 0x1F) << 3;

        image888[3*i]     = r;
        image888[3*i + 1] = g;
        image888[3*i + 2] = b;
    }
}


int main(void) {
    pci_driver_fd = open_pci_driver();
    uint16_t* image_buf = reinterpret_cast<uint16_t*>(image_buf_temp);
    if (pci_driver_fd) {
        cmd_operation(pci_driver_fd, info_num, &command_operation);
        printf("The tandem load base address is [ Bar0 ]\n");
    } else {
        printf("PCIe Device Open Fail !!!\n");
        return -1;
    }

    char input[10];
    bool continuous_capture = false;
    cv::namedWindow("Converted Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Converted Image", 800, 600);

    // 帧率计算相关变量
    int frame_count = 0;
    double total_time = 0.0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_print_time = start_time;

    while (1) {
        if (!continuous_capture) {
            printf("输入y开始连续采集，q退出: ");
            if (fgets(input, sizeof(input), stdin) == NULL) continue;
            input[strcspn(input, "\n")] = '\0';
            
            if (strcmp(input, "y") == 0) {
                continuous_capture = true;
                // 重置帧率计数器
                frame_count = 0;
                total_time = 0.0;
                start_time = std::chrono::high_resolution_clock::now();
                last_print_time = start_time;
            } else if (strcmp(input, "q") == 0) {
                break;
            }
            continue;
        }

        auto frame_start = std::chrono::high_resolution_clock::now();

        // 连续采集模式
        dma_operation.current_len = 3840/4;
        dma_operation.offset_addr = 0;
        memset(dma_operation.data.write_buf, 0, DMA_MAX_PACKET_SIZE);
        memset(dma_operation.data.read_buf, 0, DMA_MAX_PACKET_SIZE);
        
        ioctl(pci_driver_fd, PCI_MAP_ADDR_CMD, &dma_operation);
        for (int i = 0; i < 1080; i++) {
            memset(dma_operation.data.read_buf, 0, DMA_MAX_PACKET_SIZE);
            ioctl(pci_driver_fd, PCI_DMA_WRITE_CMD, &dma_operation);
            //nano_delay(1000);
			for (int k = 0;k<5000;k++);
            ioctl(pci_driver_fd, PCI_READ_FROM_KERNEL_CMD, &dma_operation);
            memcpy(image_buf_temp + i*3840, dma_operation.data.read_buf, 3840);
        }
        // 计算帧时间
        auto frame_end = std::chrono::high_resolution_clock::now();
        double frame_time = std::chrono::duration<double>(frame_end - frame_start).count();
        total_time += frame_time;
        frame_count++;
        ioctl(pci_driver_fd, PCI_UMAP_ADDR_CMD, &dma_operation);

        rgb565_to_rgb888(image_buf, image_buf_888, 1920*1080);
        
        cv::Mat img(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3, image_buf_888);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        cv::imshow("Converted Image", img);

        

        // 每1秒打印一次帧率信息
        auto current_time = std::chrono::high_resolution_clock::now();
        if (std::chrono::duration<double>(current_time - last_print_time).count() >= 1.0) {
            double avg_fps = frame_count / total_time;
            double instant_fps = 1.0 / frame_time;
            
            printf("\r平均帧率: %.2f FPS | 瞬时帧率: %.2f FPS | 总帧数: %d", 
                  avg_fps, instant_fps, frame_count);
            fflush(stdout);  // 立即刷新输出
            
            // 重置统计周期
            frame_count = 0;
            total_time = 0.0;
            last_print_time = current_time;
        }

        // 检查按键（10ms超时）
        int key = cv::waitKey(10);
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == 's' || key == 'S') {  // 可添加暂停功能
            continuous_capture = false;
            printf("\n采集已暂停\n");
        }
    }

    cv::destroyAllWindows();
    close(pci_driver_fd);
    printf("\nExit main\n");
    return 0;
}
