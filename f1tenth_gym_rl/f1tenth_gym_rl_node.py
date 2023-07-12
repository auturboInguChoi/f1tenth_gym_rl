#!/usr/bin/env python3

import gym
import collections
import random

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

#Hyperparameters
learning_rate = 0.0002
gamma         = 0.98
buffer_limit  = 50000
batch_size    = 32

device = None

class F1TenthGym():
    def __init__(self):
        print("F1TenthGym - __init__")
    
    ### reset() : 새로운 에피소드를 불러온다.
    ## - 입력 : 없음
    ## - 기능
    # 1. 로봇 위치 초기화 : /initialpose : init the pose of the robot
    # 2. 상태(State) 계산을 위한 변수 준비
        # 2-1. 라이다 센서 값 : /scan
        # 2-2. 목표 지점까지의 거리 : /tf - position, /goal_pose
        # 2-3. 목표 각도까지의 오차 각도 : /tf - orientation, /goal_pose
    ## - 출력 : 상태
    def reset():
        print("F1TenthGym - Reset")
        
        
    ### step() : 액션을 취하면 그에 대한 결과 값을 반환한다.
    ## - 입력 : 행동 (action)
    ## - 기능
    # 1. 로봇의 제어 입력 /cmd_vel : move the robot with linear & angular velocity
    ## - 출력 : 상태, 보상, 끝남 여부, 정보          
    def step():
        print("F1TenthGym - Step")        
        
    def close():
        print("F1TenthGym - Close")        


class ReplayBuffer():
    def __init__(self):
        self.buffer = collections.deque(maxlen=buffer_limit)
    
    def put(self, transition):
        self.buffer.append(transition)
    
    def sample(self, n):
        mini_batch = random.sample(self.buffer, n)
        s_lst, a_lst, r_lst, s_prime_lst, done_mask_lst = [], [], [], [], []
        
        for transition in mini_batch:
            s, a, r, s_prime, done_mask = transition
            s_lst.append(s)
            a_lst.append([a])
            r_lst.append([r])
            s_prime_lst.append(s_prime)
            done_mask_lst.append([done_mask])

        return torch.tensor(s_lst, dtype=torch.float).to(device), torch.tensor(a_lst).to(device), \
               torch.tensor(r_lst).to(device), torch.tensor(s_prime_lst, dtype=torch.float).to(device), \
               torch.tensor(done_mask_lst).to(device)
    
    def size(self):
        return len(self.buffer)

class Qnet(nn.Module):
    def __init__(self):
        super(Qnet, self).__init__()
        self.fc1 = nn.Linear(4, 128).to(device)
        self.fc2 = nn.Linear(128, 128).to(device)
        self.fc3 = nn.Linear(128, 2).to(device)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
      
    def sample_action(self, obs, epsilon):
        out = self.forward(obs)
        coin = random.random()
        if coin < epsilon:
            return random.randint(0,1)
        else : 
            return out.argmax().item()
            
def train(q, q_target, memory, optimizer):
    for i in range(10):
        s,a,r,s_prime,done_mask = memory.sample(batch_size)

        q_out = q(s)
        q_a = q_out.gather(1,a)
        max_q_prime = q_target(s_prime).max(1)[0].unsqueeze(1)
        target = r + gamma * max_q_prime * done_mask
        loss = F.smooth_l1_loss(q_a, target)
        
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()


def main():
    # torch for GPU
    if torch.cuda.is_available():
        device = 'cuda'
    else:
        device = 'cpu'

    print("torch device = ", device)    
    
    print('Hi from f1tenth_gym_rl.')
    
    ######################################
    ######################################
    # RL Training...
    ######################################
    ######################################
    
    ### env = gym.make('CartPole-v1') : 원하는 환경 생성
    env = gym.make('CartPole-v1')
    q = Qnet()

    q_target = Qnet()
  
    q_target.load_state_dict(q.state_dict())
    memory = ReplayBuffer()

    print_interval = 20
    score = 0.0  
    optimizer = optim.Adam(q.parameters(), lr=learning_rate)

    for n_epi in range(10000):
        epsilon = max(0.01, 0.08 - 0.01*(n_epi/200)) #Linear annealing from 8% to 1%

        ### env.reset() : 새로운 에피소드를 불러온다.
        ## - 입력 : 없음
        ## - 기능
        # 1. 로봇 위치 초기화 : /initialpose : init the pose of the robot
        # 2. 상태 계산
            # 2-1. 라이다 센서 값 : /scan
            # 2-2. 목표 지점까지의 거리 : /tf
            # 2-3. 목표 각도까지의 오차 각도 : /tf
        # 3. (Optional) 목표 지점 : /goal_pose
        ## - 출력 : 상태
        
        s = env.reset()
        done = False
        
        print("n_epi = ", n_epi)

        while not done:
            ### sample_action(obs, epsilon)
            a = q.sample_action(torch.from_numpy(s).float(), epsilon)      
            
            ### observation, reward, done, info = env.step(action)
            ## - 입력 : 행동 (action)
            ## - 기능
            # 1. 로봇의 제어 입력 /cmd_vel : move the robot with linear & angular velocity
            ## - 출력 : 상태, 보상, 끝남 여부, 정보            
            s_prime, r, done, info = env.step(a)
            
            done_mask = 0.0 if done else 1.0
            memory.put((s,a,r/100.0,s_prime, done_mask))
            s = s_prime

            score += r

            if done:
                break
            
        if memory.size()>2000:
            train(q, q_target, memory, optimizer)
        
        if n_epi%print_interval==0 and n_epi!=0:
            q_target.load_state_dict(q.state_dict())
            print("n_episode :{}, score : {:.1f}, n_buffer : {}, eps : {:.1f}%".format(
                                                            n_epi, score/print_interval, memory.size(), epsilon*100))
            score = 0.0
            
    ### env.close() : 환경 종료
    env.close()
    


if __name__ == '__main__':
    main()
