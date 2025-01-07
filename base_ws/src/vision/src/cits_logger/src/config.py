SIGNAL_NAME_LIST=[
    "1. 200 북서좌 14",
    "2. 300 남서직 15",
    "3. 400 남직  1",
    "4. 500 남좌 2",
    "5. 610 동좌 4",    
    "6. 300 북동직 11",
    "7. 200 북동직 11"]

SIGNAL_REGION_LIST=[
    200 ,
    300,
    400,
    500,
    610,    
    300,
    200]

SIGNAL_MARKER_POSION_LIST=[
    [0,0],
    [0,2],
    [0,4],
    [0,6],
    [-2,6],    
    [-0.5,2],
    [-0.5,0]]


SIGNAL_UPDATE_INDEX={

    200 : { "query"  : [ 14 , 11], "index" :[0,6] ,"postion" :[0,0]},
    300 : { "query"  : [ 11, 15], "index" :[1,5], "postion" :[0,1]},
    400 : { "query"  : [ 1], "index" :[2], "postion" :[0,2] },
    500 : { "query"  :  [2],"index" :[3], "postion" :[0,3] },
    610 : { "query"  :  [4],"index" :[4], "postion" :[0,4] },

    
}


SIGNAL_MEESAGE_TYPE_NUMBER =19



#recent
PASSER_PATTERN_LIST=[ 

    #recent 
    (r'\[.*\]\[.*\]\[.*\]\s-\s*length\s*: (\d+)\s*,\s*fsSec : (\d+)\s*,\s*fsUsec : (\d+)\s*,\s*dataCnt : (\d+)\s*,\s*dataLength : (\d+)'),

    #test
    (r'\[.*?\](?:\[.*?\])?\s+INFO\s+.*? - length : (\d+) , fsSec : (\d+), fsUsec : (\d+) , dataCnt : (\d+) , dataLength : (\d+)')
]

