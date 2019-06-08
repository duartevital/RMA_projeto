#include <vector>

class Question{

public:
    std::string getQuestion(){  return question; }
    int getAnswer(){    return answer; }
    Object getRandomObject();
    void setQuestion();
    void setPositionAnswer(std::vector<Object> vec);
    void setDistanceAnswer(std::vector<Object> vec);
    void setAreaAnswer(std::vector<Object> vec);
    void setHeightAnswer(std::vector<Object> vec);
    void setWidthAnswer(std::vector<Object> vec);
    void setColorAnswer(std::vector<Object> vec);
    void setTypeAnswer(std::vector<Object> vec);

    //Question(int quest, Object obj_quest);
    Question(int quest, int sub_quest);
    //Question(int quest, int sub_quest, Object obj_quest);
    Question();

private:
    int quest, sub_quest, answer;
    std::string question;
    std::vector<int> possible_answers;
    Object obj_quest;
};

/*Question::Question(int quest, Object obj_quest){
    this->quest = quest;
    this->obj_quest = obj_quest;

    setQuestion();
}*/

Question::Question(int quest, int sub_quest){
    this->quest = quest;
    this->sub_quest = sub_quest;
    obj_quest = getRandomObject();
    std::cout << "Obj Selecionado: " << obj_quest.toString() << std::endl;

    setQuestion();
}

/*Question::Question(int quest, int sub_quest, Object obj_quest){
    this->quest = quest;
    this->sub_quest = sub_quest;
    this->obj_quest = obj_quest;

    setQuestion();
}*/

Question::Question(){   }

void Question::setQuestion(){
    switch(quest){
        case 1:
            if(sub_quest == 1)
                question = "Selecciona um objeto à direita do objeto pré-seleccionado.";
            else if (sub_quest == 2)
                question = "Selecciona um objeto à esquerda do objeto pré-seleccionado.";
            break;
        case 2:
            if(sub_quest == 1)
                question = "Selecciona o objeto mais próximo do objeto pré-seleccionado.";
            else if (sub_quest == 2)
                question = "Selecciona o objeto mais afastado do objeto pré-seleccionado.";
            setDistanceAnswer(objects_in_cloud);
            break;
        case 3:
            if(sub_quest == 1)
                question = "Selecciona o objeto com maior superfície.";
            else if (sub_quest == 2)
                question = "Selecciona o objeto com menor superfície.";
            setAreaAnswer(objects_in_cloud);
            break;
        case 4:
            if(sub_quest == 1)
                question = "Selecciona o objeto mais alto.";
            else if (sub_quest == 2)
                question = "Selecciona o objeto mais baixo.";
            setHeightAnswer(objects_in_cloud);
            break;
        case 5:
            if(sub_quest == 1)
                question = "Selecciona o objeto mais largo";
            else if (sub_quest == 2)
                question = "Selecciona o objeto mais estreito.";
            setWidthAnswer(objects_in_cloud);
            break;
        case 6:
            if(sub_quest == 1)
                question = "Seleciona um objeto mais escuro do que o objeto pré-seleccionado";
            else if (sub_quest == 2)
                question = "Seleciona um objeto mais claro do que o objeto pré-seleccionado.";
            setColorAnswer(objects_in_cloud);
            std::cout << "\n" << std::endl;
            for(int i=0; i<possible_answers.size(); i++){
                std::cout << objects_in_cloud[possible_answers[i]].toString() << std::endl;
            }
            break;
        case 7:
            std::stringstream s;
            s << "Selecciona o objeto do tipo " << obj_quest.getType();
            question = s.str();
            setTypeAnswer(objects_in_cloud);
            break;
    }
}

Object Question::getRandomObject(){
    int r = rand() % objects_in_cloud.size();
    return objects_in_cloud[r];
}


//The next functions return an asnwer for each pair of questions
//This answer is represented by an Integer, wich corresponds to a position on objects_in_file

void Question::setPositionAnswer(std::vector<Object> vec){

}

void Question::setDistanceAnswer(std::vector<Object> vec){
    int pos = 0;
    if(sub_quest == 2){
        int max_distance = 0;
        for(int i=0; i<vec.size(); i++){
            float d = getEuclideanDistance(vec[i].getCentroid(), obj_quest.getCentroid());
            if(d > max_distance){
                max_distance = d;
                pos = i;
            }
        }
        answer = pos;

    }else if(sub_quest == 1){
        int min_distance = 10000;
        for(int i=0; i<vec.size(); i++){
            float d = getEuclideanDistance(vec[i].getCentroid(), obj_quest.getCentroid());
            if(d < min_distance && d > 0){
                min_distance = d;
                pos = i;
            }
        }
        answer = pos;
    }
}

void Question::setAreaAnswer(std::vector<Object> vec){
    int pos = 0;
    if(sub_quest == 1){
        int max_area = 0;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getArea() > max_area){
                max_area = vec[i].getArea();
                pos = i;
            }
        answer = pos;

    }else if(sub_quest == 2){
        int min_area = 1000;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getArea() < min_area){
                min_area = vec[i].getArea();
                pos = i;
            }
        answer = pos;
    }
}

void Question::setHeightAnswer(std::vector<Object> vec){
    int pos = 0;
    if(sub_quest == 1){
        int max_height = 0;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getHeight() > max_height){
                max_height = vec[i].getHeight();
                pos = i;
            }
        answer = pos;

    }else if(sub_quest == 2){
        int min_height = 1000;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getHeight() < min_height){
                min_height = vec[i].getHeight();
                pos = i;
            }
        answer = pos;
    }
}

void Question::setWidthAnswer(std::vector<Object> vec){
    int pos = 0;
    if(sub_quest == 1){
        int max_width = 0;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getWidth() > max_width){
                max_width = vec[i].getWidth();
                pos = i;
            }
        answer = pos;

    }else if(sub_quest == 2){
        int min_width = 1000;
        for(int i=0; i<vec.size(); i++)
            if(vec[i].getWidth() < min_width){
                min_width = vec[i].getWidth();
                pos = i;
            }
        answer = pos;
    }
}

void Question::setColorAnswer(std::vector<Object> vec){
    if(sub_quest == 2){
        while(possible_answers.size()==0){
            for(int i=0; i<vec.size(); i++)
                if(vec[i].getAvgColor() > obj_quest.getAvgColor())
                    possible_answers.push_back(i);

            if(possible_answers.size()==0){
                obj_quest = getRandomObject();
                //std::cout << "Obj Selecionado: " << obj_quest.toString() << std::endl;
            }
        }
    }else if(sub_quest == 1){
        while(possible_answers.size()==0){
            for(int i=0; i<vec.size(); i++)
                if(vec[i].getAvgColor() < obj_quest.getAvgColor())
                    possible_answers.push_back(i);

            if(possible_answers.size()==0){
                obj_quest = getRandomObject();
                //std::cout << "Obj Selecionado: " << obj_quest.toString() << std::endl;
            }
        }
    }
}

void Question::setTypeAnswer(std::vector<Object> vec){
    for(int i=0; i<vec.size(); i++){
        if(vec[i].getType().compare(obj_quest.getType()) == 0)
            possible_answers.push_back(i);
    }
}
