# Collaboration Guide - 협업 가이드

Private GitHub 저장소를 다른 사람과 공유하는 방법입니다.

---

## Private 저장소 공유 방법

현재 저장소는 **Private**로 설정되어 있어, 기본적으로 본인만 접근할 수 있습니다.

### ❌ 불가능한 것
```bash
# 다른 사람이 이 명령을 실행하면 에러 발생
git clone https://github.com/WoonBong/apriltag_navigation.git
# Error: Repository not found (권한 없음)
```

---

## 방법 1: Collaborator 추가 (추천)

다른 사람에게 저장소 접근 권한을 부여하는 방법입니다.

### 단계

1. **GitHub 저장소 페이지로 이동**
   - https://github.com/WoonBong/apriltag_navigation

2. **Settings 클릭**
   - 저장소 페이지 상단의 "Settings" 탭

3. **Collaborators 메뉴**
   - 왼쪽 메뉴에서 "Collaborators and teams" 클릭

4. **Add people 버튼**
   - "Add people" 클릭

5. **사용자 추가**
   - 협업자의 GitHub 아이디 또는 이메일 입력
   - 예: `john-doe` 또는 `john@example.com`

6. **초대 전송**
   - "Add john-doe to this repository" 클릭
   - 상대방에게 이메일 초대가 전송됨

7. **상대방이 수락**
   - 초대받은 사람이 이메일에서 "Accept invitation" 클릭

### 권한 레벨

- **Read**: 읽기만 가능 (clone, pull)
- **Write**: 읽기 + 쓰기 가능 (push, commit)
- **Admin**: 모든 권한 (설정 변경 가능)

### 이후 Clone 가능

```bash
# 협업자가 clone 가능
git clone https://github.com/WoonBong/apriltag_navigation.git
```

---

## 방법 2: 저장소를 Public으로 변경

누구나 접근할 수 있도록 공개하는 방법입니다.

### ⚠️ 주의사항
- **모든 사람**이 코드를 볼 수 있습니다
- **비밀번호, API 키 등 민감한 정보**가 없는지 확인 필수!

### 단계

1. **GitHub 저장소 Settings**
   - https://github.com/WoonBong/apriltag_navigation/settings

2. **Danger Zone 스크롤**
   - 페이지 맨 아래로 스크롤

3. **Change visibility**
   - "Change visibility" 클릭

4. **Make public**
   - "Make public" 선택
   - 경고 메시지 읽기
   - 저장소 이름 입력하여 확인

### 이후 누구나 Clone 가능

```bash
# 누구나 clone 가능
git clone https://github.com/WoonBong/apriltag_navigation.git
```

---

## 방법 3: 특정 브랜치만 공유 (고급)

Main 브랜치는 Private으로 유지하고, 특정 브랜치만 공개하는 방법은 GitHub에서 직접 지원하지 않습니다.

대신 **별도의 Public 저장소를 만들어 동기화**할 수 있습니다:

```bash
# 별도의 public 저장소 생성
gh repo create apriltag-navigation-public --public

# Public 저장소를 remote로 추가
git remote add public https://github.com/WoonBong/apriltag-navigation-public.git

# 특정 브랜치만 push
git push public develop:main
```

---

## 방법 4: ZIP 파일로 공유

GitHub를 사용하지 않고 코드를 직접 전달하는 방법입니다.

### 단계

1. **저장소에서 ZIP 다운로드**
   ```bash
   cd /home/hyf/nav_ws/src/apriltag_navigation

   # Git 히스토리 제외하고 압축
   tar -czf apriltag_navigation.tar.gz \
       --exclude='.git' \
       --exclude='__pycache__' \
       --exclude='*.pyc' \
       .
   ```

2. **파일 전송**
   - 이메일, USB, 클라우드 드라이브 등으로 전송

3. **받는 사람이 압축 해제**
   ```bash
   tar -xzf apriltag_navigation.tar.gz
   cd apriltag_navigation
   ```

**단점:**
- Git 히스토리가 없음
- 변경사항 동기화 어려움

---

## 추천 방법 비교

| 방법 | 장점 | 단점 | 추천 상황 |
|------|------|------|-----------|
| **Collaborator 추가** | 권한 제어 가능<br>Private 유지<br>Git 기능 완전 활용 | 상대방 GitHub 계정 필요 | **팀 프로젝트<br>신뢰할 수 있는 협업자** |
| **Public 저장소** | 누구나 접근 가능<br>오픈소스 기여 가능 | 모든 코드 공개<br>민감 정보 주의 | 오픈소스 프로젝트<br>포트폴리오 |
| **ZIP 파일** | GitHub 불필요<br>간단함 | Git 기능 없음<br>동기화 어려움 | 일회성 공유<br>빠른 시연 |

---

## 현재 저장소 상태 확인

```bash
# 저장소가 Public인지 Private인지 확인
gh repo view WoonBong/apriltag_navigation --json visibility
# 결과: "visibility": "PRIVATE"

# Collaborator 목록 확인
gh api repos/WoonBong/apriltag_navigation/collaborators
```

---

## Collaborator 추가 실습 (CLI 사용)

GitHub CLI로도 가능합니다:

```bash
# Collaborator 추가
gh api -X PUT repos/WoonBong/apriltag_navigation/collaborators/USERNAME \
  -f permission=write

# 예시
gh api -X PUT repos/WoonBong/apriltag_navigation/collaborators/john-doe \
  -f permission=write
```

---

## 공유 후 협업 워크플로우

### 협업자가 처음 시작할 때

```bash
# 1. Clone
git clone https://github.com/WoonBong/apriltag_navigation.git
cd apriltag_navigation

# 2. 브랜치 확인
git branch -a
# main
# develop

# 3. Develop 브랜치로 전환
git checkout develop

# 4. ROS 빌드
cd ~/catkin_ws  # 또는 적절한 워크스페이스
catkin_make
source devel/setup.bash
```

### 협업자가 작업할 때

```bash
# 1. 최신 코드 받기
git pull origin develop

# 2. 새 브랜치 생성
git checkout -b feature/my-feature

# 3. 작업 후 커밋
git add .
git commit -m "Add my feature"

# 4. Push
git push origin feature/my-feature

# 5. GitHub에서 Pull Request 생성
# develop <- feature/my-feature
```

### 당신(저장소 소유자)이 리뷰할 때

```bash
# GitHub 웹에서 Pull Request 확인
# "Merge pull request" 클릭하여 병합

# 또는 CLI로
gh pr list
gh pr review 1 --approve
gh pr merge 1
```

---

## 보안 체크리스트

Public으로 만들기 전에 반드시 확인:

- [ ] 비밀번호가 코드에 없는가?
- [ ] API 키가 없는가?
- [ ] 개인 정보 (이메일, 전화번호)가 없는가?
- [ ] 회사/학교 민감 정보가 없는가?
- [ ] `.gitignore`가 적절히 설정되었는가?

```bash
# 민감한 정보 검색
grep -r "password" .
grep -r "api_key" .
grep -r "@" . --include="*.py"  # 이메일 검색
```

---

## 요약

**가장 추천하는 방법: Collaborator 추가**

1. GitHub 저장소 → Settings → Collaborators
2. "Add people" 클릭
3. 협업자의 GitHub 아이디 입력
4. 초대 전송 → 상대방이 수락
5. 상대방이 `git clone` 가능!

간단하고 안전하며, Git의 모든 기능을 사용할 수 있습니다!
