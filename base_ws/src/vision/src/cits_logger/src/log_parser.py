import re
import json
import datetime
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from config import *

def timestamp_to_standard_time(timestamp_str):
    try:
        # 문자열을 정수로 변환
        timestamp = int(timestamp_str)
        
        # 타임스탬프를 datetime 객체로 변환
        dt_object = datetime.datetime.fromtimestamp(timestamp)
        
        # datetime 객체를 원하는 형식의 문자열로 변환
        formatted_time = dt_object.strftime("%Y-%m-%d %H:%M:%S")
        
        return formatted_time
    except ValueError:
        return "잘못된 입력입니다. 유효한 타임스탬프를 입력해주세요."


def parse_log_and_json(signal_state, pub, pattern_number):

   
    # 첫 번째 줄의 로그 입력 받기
    try: 

        log_info_match=True
    
        while(log_info_match):
            # print("problem")
            log_line = input()
            
            pattern_log_info = PASSER_PATTERN_LIST[pattern_number]
            
            log_info_match = re.search(pattern_log_info, log_line)
            if log_info_match:
                break

    except EOFError as e:
        print(f"[CITS] wait info.. {e}")
        return signal_state
    
    


  
    if log_info_match:
        log_info = {
            "length": int(log_info_match.group(1)),
            "fsSec": int(log_info_match.group(2)),
            "fsUsec": int(log_info_match.group(3)),
            "dataCnt": int(log_info_match.group(4)),
            "dataLength": int(log_info_match.group(5))
        }
        
        if log_info["dataCnt"] ==SIGNAL_MEESAGE_TYPE_NUMBER:    
         
            # JSON 데이터 입력 받기
            json_lines = []
            json_line=" "
            # for i in range(data_length+1):
            try:
                first_line =input()
                while(json_line[0] != '}'):
                    json_line = input()
                    # print(json_line)
                    json_lines.append(json_line)
                    if(json_line[0]=='['):
                        json_lines=[]
                        input()
                    
            
            except Exception as e:
                # rospy.logerr(f"ERROR {e}")
                print("[CITS] wait info.. {e}")
                return signal_state
            
            # 전체 JSON 데이터를 하나의 문자열로 합치기
                
            # print("========================================================")

         
            pattern_json = r'{'  
            json_match = re.search(pattern_json, first_line)
            first_str = first_line[json_match.start():] # JSON 문자열만 추출
            json_str = "\n".join( [first_str] + json_lines )


            
            # JSON 파싱 시도
            try:
                json_data = json.loads(json_str)
            
                
                # 로그 정보와 JSON 데이터를 합침
                combined_data = {
                    "log_info": log_info,
                    "json_data": json_data
                }
                # print( timestamp_to_standard_time(log_info["fsSec"]),json_data["value"]["intersections"][0]["id"]["region"])

                region =json_data["value"]["intersections"][0]["id"]["region"]
                states =json_data["value"]["intersections"][0]["states"]


                if int(region) in SIGNAL_UPDATE_INDEX.keys():
                    for query, index in zip(SIGNAL_UPDATE_INDEX[region]["query"],SIGNAL_UPDATE_INDEX[region]["index"]):
                        if(states[query-1]["state-time-speed"][0]["eventState"]=="protected-Movement-Allowed"):
                            signal_state[index*2] = 1
                            signal_state[index*2+1] = int(states[query-1]["state-time-speed"][0]["timing"]["minEndTime"])
                        else:
                            signal_state[index*2] = 0
                            signal_state[index*2+1] = int(states[query-1]["state-time-speed"][0]["timing"]["minEndTime"])
                        # else:
                        #     print(states[query-1]["state-time-speed"][0]["eventState"])
                        #     print(int(states[query-1]["state-time-speed"][0]["timing"]["minEndTime"]))

                
                print(log_line)
                for i in range(0,7):
                    print(SIGNAL_NAME_LIST[i],signal_state[i*2] ,signal_state[i*2+1])
                
                msg = Int32MultiArray()
                msg.data = signal_state
                msg.layout.data_offset=log_info["fsSec"]
                pub.publish(msg)

            
            except json.JSONDecodeError as e:
                print(f"JSON 파싱 오류: {e}")
                rospy.logerr("ERROR")
                return signal_state
  
        
    return signal_state
