#include "mcp2515.h"

struct command {
  String message;
  bool isRequest;
  int command_num;
  int data;
  int from_desk_id;
  int to_desk_id;
};

namespace ch {
  enum msg_type{
      I_ = 1,
      I_VAL_ = 2,
      X_I_ = 3,
      ACKERR = 4,
      STREAM = 5,
      BUFFER = 6
  };
  int x_type[2][32] = {
    {
      4,2,4,2,
      2,4,2,4,
      2,4,2,2,
      2,2,5,4,
      6,2,2,2,
      2,4,2,4,
      2,2,4,4,
      0,2,0,0
    },
    {
      2,1,2,1,
      1,2,1,2,
      1,3,1,1,
      1,1,3,3,
      3,1,1,1,
      1,2,1,2,
      1,1,2,0,
      1,0,0,0
    }
  };

  char commands[2][32][5] {
  {
      ""    , "d "  , ""    , "r "  ,
      "l "  , ""    , "o "  , ""    ,
      "a "  , ""    , "k "  , "x "  ,
      "p "  , "t "  , "s "  , ""    ,
      "b "  , "e "  , "v "  , "f "  ,
      "O "  , ""    , "U "  , ""    ,
      "L "  , "c "  , ""    , ""    ,
      ""    , "C "    , ""    , ""   
  },
  {
      "d "  , "g d ", "r "  , "g r ",
      "g l ", "o "  , "g o ", "a "  ,
      "g a ", "k "  , "g k ", "g x ",
      "g p ", "g t ", "s "  , "S "  ,
      "g b ", "g e ", "g v ", "g f ",
      "g O ", "O "  , "g U ", "U "  ,
      "g L ", "g c ", "c "  , "r"   ,
      "w "  , "C"   , " "   , " "
  }
  };

  
  
  can_frame command2frame(command can_command){
    can_frame canMsgTx;
    
    bool isRequest = can_command.isRequest;
    String message = can_command.message;
    int data = can_command.data;
    int from_desk_id = can_command.from_desk_id;
    int to_desk_id = can_command.to_desk_id;

    int commandNum = can_command.command_num;

    bool sp_flag = (isRequest && (commandNum >= 15 && commandNum <= 17) && (message[2] == 'l'))
                  || (x_type[isRequest][commandNum] == ACKERR) && (message[0] == 'a');

    canMsgTx.can_id =  1024 * (sp_flag) +
                       512 * (from_desk_id > 1) +
                       256 * (from_desk_id % 2) +
                       128 * (to_desk_id > 1) +
                       64 * (to_desk_id % 2) +
                       32 * isRequest +
                       commandNum;

    int can_dlc = 0;
    int messageData[8] {0};

    for(int i = 0; i < 8; i++){
      if (data == 0) break;
      int two_pow = 1;
      for(int j = 0; j < 8; j++){
        messageData[i] += two_pow * (data % 2);
        data /= 2;
        two_pow *= 2;
      }
      can_dlc++;
    }

    canMsgTx.can_dlc = can_dlc;        
    for(int i = 0; i < can_dlc; i++) canMsgTx.data[i] = messageData[i];

    return canMsgTx;
  }

  command frame2command(can_frame canMsgRx){
    command can_command{"None",0,0,0};
    
    int message_id = canMsgRx.can_id;

    bool sp_flag = message_id / 1024;
    message_id %= 1024;

    int from_desk_id = 2*(message_id / 512) + (message_id % 512) / 256;
    message_id %= 512;
    message_id %= 256;
    
    int to_desk_id = 2*(message_id / 128) + (message_id % 128) / 64;
    message_id %= 128;
    message_id %= 64;

    bool isRequest = message_id / 32;
    message_id %= 32;
      
    if (message_id < 0 || message_id >= 32){
        return can_command;
    }

    String msg = commands[isRequest][message_id];
    int msg_type = x_type[isRequest][message_id];
    
    String space = " ";

    int data = 0;
    auto can_data = canMsgRx.data;

    int can_dlc = canMsgRx.can_dlc;

    for(int i = 0; i < can_dlc; i++) {
        data |= can_data[i] << (8 * i);
    }

    if (msg_type == I_){
      msg = msg + to_desk_id;
    } 
    
    else if (msg_type == I_VAL_){
      msg = msg + to_desk_id + space + data;
    } 
    
    else if (msg_type == X_I_){
      char x;

      if (sp_flag) x = 'l';
      else x = 'd';

      msg = msg + x + space + to_desk_id;
    } 

    else if (msg_type == ACKERR){
      if (sp_flag) msg = "ack";
      else msg = "err";
    }

//    else if (msg_type == STREAM){
//      
//    }
//
//    else if (msg_type == BUFFER){
//      
//    }
//    
    else if (msg_type == 0) {
       msg = "r";
    }

    else {
       msg = "MSG_ERROR";
    }

    can_command.message = msg;
    can_command.data = data;
    can_command.isRequest = isRequest;
    can_command.command_num = message_id;
    can_command.from_desk_id = from_desk_id;
    can_command.to_desk_id = to_desk_id;

    return can_command;
  }
}
