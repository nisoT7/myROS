/* ros用ヘッダファイル */
#include<ros/ros.h>
#include<ros/package.h>
#include<std_msgs/String.h>
/* julius用ヘッダファイル */
#include<julius/juliuslib.h>
/* 一般ヘッダファイル */
#include<stdio.h>
#include<sstream>
#include<string>
/* 文字列変換用ヘッダファイル */
#include<iconv.h>

#define STR_OUT 1000

using namespace std;

namespace{
    ros::Publisher speech_pub;
}

class charconv{
    private:
        iconv_t ic;
    public:
        charconv();
        void convert(char *str_in, char *str_out, size_t str_out_size);
        ~charconv();
};

charconv::charconv(){
    ic= iconv_open("UTF-8","EUC-JP");
}

charconv::~charconv(){
    iconv_close(ic);
}

void charconv::convert(char *str_in, char *str_out, size_t str_out_size){
    char *ptr_in = str_in;
    char *ptr_out = str_out;
    size_t inbufsz = strlen(str_in)+1;
    size_t outbufsz = str_out_size-1;
    iconv(ic, &ptr_in, &inbufsz, &ptr_out, &outbufsz);
}

void status_recready(Recog *recog, void *dummy){
    if(recog->jconf->input.speech_input == SP_MIC || recog->jconf->input.speech_input == SP_NETAUDIO){
        fprintf(stderr,"<<< please speak >>>\n");
    }
}

static void output_result(Recog *recog, void *dummy){
    int len;
    WORD_INFO *winfo;
    WORD_ID *seq;
    int seqnum;
    int n;
    char str_out[STR_OUT];
    charconv *cc = new charconv;
    Sentence *s;
    RecogProcess *r;
    std_msgs::String msg;
    stringstream ss;

    if(!ros::ok()){
        exit(0);
    }

    for(r = recog->process_list; r; r = r->next){
        if(!r->live) continue;
        if(r->result.status < 0) continue;
        winfo = r->lm->winfo;

        for(n = 0; n < r->result.sentnum; n++){
            s = &(r->result.sent[n]);
            seq = s->word;
            seqnum = s->word_num;

            printf("認識結果：",n+1);
            for(int i = 0; i < seqnum; i++){
                cc->convert(winfo->woutput[seq[i]], str_out, (size_t)STR_OUT);
                printf("%s",str_out);
            }
        }
    }
    ss << str_out;
    msg.data = ss.str();
    delete cc;
    fflush(stdout);
    speech_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"speech");
    ros::NodeHandle node;

    Jconf *jconf;
    Recog *recog;
    int ret;
    static char speechfilename[MAXPATHLEN];
    string param1;
    string param2;
    int numargs = 3;
    char *arguments[numargs];

    speech_pub = node.advertise<std_msgs::String>("speech",100);

    /* logファイルを出さない */
    jlog_set_output(NULL);

    /* 第一引数のチェック */
    if(argc == 1){
        ros::NodeHandle nh("~");
        nh.getParam("config",param1);
        nh.getParam("configfile",param2);
        if(param1 =="" || param2 ==""){
            param1 = "-C";
            param2 = ros::package::getPath("speech")+"/Sample.jconf";
            arguments[0] = argv[0];
            arguments[1] = (char *)param1.c_str();
            arguments[2] = (char *)param2.c_str();
        }
        ROS_WARN("Config file : %s",param2.c_str());
        arguments[0] = argv[0];
        arguments[1] = (char *)param1.c_str();
        arguments[2] = (char *)param2.c_str();
        jconf = j_config_load_args_new(numargs,arguments);
    }
    else if(argc == 2)
    {
        param1 = "-C";
        param2 = argv[1];
        arguments[0] = argv[0];
        arguments[1] = (char *)param1.c_str();
        arguments[2] = (char *)param2.c_str();
        ROS_WARN("Config file : %s",param2.c_str());
        jconf = j_config_load_args_new(numargs,arguments);
    }
    else if(argc == 3)
    {
        param2 = argv[2];
        ROS_WARN("Config file : %s",param2.c_str());
        jconf = j_config_load_args_new(argc, argv);
    }
    else
    {
        jconf = j_config_load_args_new(argc, argv);
    }

    /* jconfファイルの有無の確認 */
    if(jconf == NULL){
        fprintf(stderr,"No jconf file.\n");
        return -1;
    }

    /* 認識器の作成 */
    recog = j_create_instance_from_jconf(jconf);
    if(recog == NULL){
        fprintf(stderr,"No recognition.\n");
        return -1;
    }

    /* コールバック */
    callback_add(recog, CALLBACK_EVENT_SPEECH_READY, status_recready, NULL);
    callback_add(recog, CALLBACK_RESULT, output_result, NULL);

    /* マイク入力の確認 */
    if(j_adin_init(recog) == FALSE) return -1;

    switch(j_open_stream(recog, NULL)){
        case 0:
            break;
        case -1:
            fprintf(stderr,"error in input stream\n");
            return 0;
        case -2:
            fprintf(stderr,"failed to begin input stream\n");
            return 0;
    }
    
    ret = j_recognize_stream(recog);
    if(ret == -1) return -1;

    ros::spin();
    j_close_stream(recog);
    j_recog_free(recog);
    return 0;
}
