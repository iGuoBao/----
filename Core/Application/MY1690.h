#ifndef __MY1690_H
#define __MY1690_H

#define MY_Play         0x11 //播放
#define MY_Pause        0x12 //暂停
#define MY_SongDown     0x13 //下一曲
#define MY_SongUp       0x14 //上一曲
#define MY_VolumeUp     0x15 //音量加
#define MY_VolumeDown   0x16 //音量减
#define MY_Reset        0x19 //复位
#define MY_FastForward  0x1A //快进
#define MY_FastBack     0x1B //快退
#define MY_PlayOrPause  0x1C //播放/暂停
#define MY_Stop         0x1E //停止

#define MY_PlayStatus   0x20 //查询播放状态              0(停止)1(播放) 2(暂停) 3(快进)4(快退) 
#define MY_VolumeLevel  0x21 //查询音量大小              0-30
#define MY_CurrentEQ    0x22 //查询当前 EQ               0-5(NO\POP\ROCK\JAZZ\CLASSIC\BASS
#define MY_PlayMode     0x23 //查询当前播放模式           0-4(全盘/文件夹/单曲/随机/无循环)
#define MY_Version      0x24 //查询版本号                 1.0
#define MY_FileNumSD    0x25 //查询SD卡的总文件数         1-65535
#define MY_FileNumU     0x26 //查询U盘的总文件总数        1-65535
#define MY_Device       0x28 //查询当前播放设备           0（U盘），1（SD卡）
#define MY_SongNumSD    0x29 //查询SD卡的当前曲目         1-65536
#define MY_SongNumU     0x2A //查询U盘的当前曲目          1-65536
#define MY_SongPlayTime 0x2C //查询当前播放歌曲的时间     反回时间（秒）
#define MY_SongTime     0x2D //查询当前播放歌曲总时间     反回时间（秒）
#define MY_SongName     0x2E //查询当前播放歌曲歌名       反回歌曲名（只能返回前两位数）
#define MY_FileSongNum  0x2F //查询当前播放文件夹内总数量  0-65536 

#define MY_SetVolume    0x31 //设置音量                   0-30级可调 
#define MY_SetEQ        0x32 //设置EQ                     0-5(NO\POP\ROCK\JAZZ\CLASSIC\BASS) 
#define MY_SetCycleMode 0x33 //设置循环模式               0-4(全盘/文件夹/单曲/随机/不循环) 
#define MY_SwitchFolder 0x34 //文件夹切换                 0（上一文件夹），1(下一文件夹)  
#define MY_SwitchDevice 0x35 //设备切换                   0（U盘），1（SD卡）
#define MY_PullUpADKEY  0x36 //ADKEY软件上拉              1开上拉（10K电阻），0关上拉，默认0 
#define MY_EnableADKEY  0x37 //ADKEY使能                  1开起，0关闭，默认1 
#define MY_SwitchBUSY   0x38 //BUSY电平切换               1为播放输出高电平，0为播放输出低电平，默认1

#define MY_SelectSong   0x41 //选择播放曲目                1-最大首目
#define MY_SelectFolder 0x42 //指定文件夹曲目播放           高八位为文件夹号(00-99)，低八位为歌曲名字(001-255)  
#define MY_InsertSong   0x43 //插播功能                    1-最大首目 
#define MY_InsertFolder 0x44 //插播指定文件夹里面的歌曲播放 高八位为文件夹号(00-99)，低八位为歌曲名字(001-255)  

void MY1690_WriteCommand(uint8_t command);
void MY1690_Read(uint8_t command, uint8_t* pData);
void MY1690_SetCommand(uint8_t command, uint8_t num);
void MY1690_LongCommand(uint8_t command, uint8_t num1, uint8_t num2);
#endif

