#include <vector>
#include <functional>
#include <thread>
#include <atomic>
#include <iomanip>

#include "ImNotification.h"
#include "icon.h"




// 알림센터 벡터
std::vector<ImGui_Notification> NOTIFICATIONS;




void ImGui::push_info_noti(const std::string &title, const std::string &content)
{
    NOTIFICATIONS.emplace_back(title, content, ImGui_NotificationType::Info);
}



void ImGui::push_sucesses_noti(const std::string &title, const std::string &content)
{
    NOTIFICATIONS.emplace_back(title, content, ImGui_NotificationType::Suceess);
}



void ImGui::push_error_noti(const std::string &title, const std::string &content)
{
    NOTIFICATIONS.emplace_back(title, content, ImGui_NotificationType::Error);
}



void ImGui::NotificationCenter()
{
    // 색상 파레트
    constexpr ImColor borderColor = ImColor(0.78f, 0.78f, 0.78f, 0.0f);     // No border
    constexpr ImColor textColor = ImColor(0.95f, 0.95f, 0.95f, 1.0f);       // White text
    constexpr ImColor subTextColor = ImColor(0.8f, 0.8f, 0.8f, 1.0f);       // Gray time
    constexpr ImColor iconColor = ImColor(0.39f, 0.59f, 1.0f, 1.0f);        // Blue circle
    constexpr ImColor bgColor_green = ImColor(0.18f, 0.8f, 0.44f, 0.7f);    // 산뜻한 에메랄드 그린 (Emerald Green)
    constexpr ImColor bgColor_red = ImColor(0.9f, 0.2f, 0.2f, 0.7f);        // 경고/에러를 위한 다홍색 계열 (Crimson Red)
    constexpr ImColor bgColor_info = ImColor(0.0f, 0.0f, 0.0f, 0.5f);       // Light gray


    // 알림센터 관련 상수
    constexpr float kShowDuration = 5.0f;       // 유지시간
    constexpr float kFadeDuration = 2.0f;       // 페이드 시간

    constexpr float kPadding = 8.0f;
    constexpr float kIconSize = 26.0f;
    constexpr float kSpacingY = 6.0f;
    constexpr float kHeaderHeight = 30.0f;
    constexpr float kRounding = 10.0f;

    constexpr float kCardWidth = 330.0f;                    // 카드 넓이
    constexpr float kMargin = 20.0f;                        // 왼쪽 마진

    constexpr ImVec2 TITLE_OFFSET = ImVec2(8.f, 0.f);       // 제목 오프셋
    constexpr ImVec2 CONTENT_OFFSET = ImVec2(12.f, -2.f);   // 내용 오프셋
    constexpr ImVec2 ICON_OFFSET = ImVec2(0.f, -3.f);       // 아이콘 오프셋




    // 알림센터 위치와 크기
    auto* viewport = ImGui::GetMainViewport();
    auto* drawList = ImGui::GetForegroundDrawList();
    ImVec2 basePos = viewport->WorkPos;
    ImVec2 workSize = viewport->WorkSize;



    // 알림 센터 표시 위치
    float posX = basePos.x + workSize.x - kCardWidth - kMargin;
    float posY = basePos.y + workSize.y - kMargin;



    // 현재 시간
    auto now = std::chrono::steady_clock::now();



    // 알림 그리기
    for (int i = static_cast<int>(NOTIFICATIONS.size()) - 1; i >= 0; --i)
    {
        auto& notif = NOTIFICATIONS[i];


        // 유지 시간 및 페이드 시간 업데이트
        float lifetime = std::chrono::duration<float>(now - notif.creationTime).count();
        float alpha = 1.0f;
        if (lifetime > kShowDuration)
        {
            float fade = lifetime - kShowDuration;
            alpha = 1.0f - (fade / kFadeDuration);


            // 시간이 다 되면 알림 벡터에서 데이터 제거
            if (alpha <= 0.0f)
            {
                NOTIFICATIONS.erase(NOTIFICATIONS.begin() + i);
                continue;
            }
        }


        // 타입에 따른 색상, 아이콘 설정
        ImColor bg;
        const char* iconText;

        if (notif.type == ImGui_NotificationType::Info)
        {
            bg = bgColor_info; bg.Value.w *= alpha;
            iconText = ICON_MD_NOTIFICATIONS;
        }

        else if (notif.type == ImGui_NotificationType::Suceess)
        {
            bg = bgColor_green; bg.Value.w *= alpha;
            iconText = ICON_MD_CHECK;
        }

        else if (notif.type == ImGui_NotificationType::Error)
        {
            bg = bgColor_red; bg.Value.w *= alpha;
            iconText = ICON_MD_PRIORITY_HIGH;
        }




        // 페이드 시간에 따라 alpha 값 업데이트
        ImColor border = borderColor; border.Value.w *= alpha;
        ImColor text = textColor;     text.Value.w *= alpha;
        ImColor subText = subTextColor; subText.Value.w *= alpha;
        ImColor icon = iconColor;     icon.Value.w *= alpha;


        // 알람 카드 크기 계산
        ImVec2 textSize = ImGui::CalcTextSize(notif.content.c_str(), nullptr, false, kCardWidth - kPadding * 2);
        float cardHeight = kHeaderHeight + textSize.y + kPadding * 3;

        ImVec2 drawMin(posX + notif.swipeOffset, posY - cardHeight);
        ImVec2 drawMax(posX + kCardWidth + notif.swipeOffset, posY);



        // 아이콘 중심 위치, 내용 시작 위치 계산
        ImVec2 contentStart = drawMin + ImVec2(kPadding, kPadding);
        ImVec2 iconCenter = contentStart + ImVec2(kIconSize * 0.5f, kIconSize * 0.5f);


        // 아이콘 크기, 위치 계산
        ImVec2 iconTextSize = ImGui::CalcTextSize(iconText);

        ImVec2 iconTextPos = ImVec2(
            iconCenter.x - iconTextSize.x * 0.5f + ICON_OFFSET.x,
            iconCenter.y - iconTextSize.y * 0.5f + ICON_OFFSET.y
        );


        // 제목 위치 계산
        float titleX = contentStart.x + kIconSize + TITLE_OFFSET.x;
        float titleY = contentStart.y + TITLE_OFFSET.y;


        // 시간이 표시될 크기 계산
        ImVec2 timeSize = ImGui::CalcTextSize(notif.timestamp.c_str());



        // 내용 위치 계산
        float bodyY = contentStart.y + kHeaderHeight;

        const float wrapWidth = kCardWidth - kPadding * 2;
        const char* text2 = notif.content.c_str();
        const char* textEnd = text2 + notif.content.size();
        const float lineHeight = ImGui::GetFontSize() + 2.0f;
        float y = bodyY;




        // 카드 배경 그리기
        drawList->AddRectFilled(drawMin, drawMax, bg, kRounding);
        drawList->AddRect(drawMin, drawMax, border, kRounding);



        // 아이콘, 제목 볼드
        ImGui::PushFont(ImGui::Bold);

        // 아이콘 그리기
        drawList->AddText(iconTextPos, IM_COL32(255, 255, 255, static_cast<int>(255 * alpha)), iconText);
        // 제목 그리기
        drawList->AddText(ImVec2(titleX, titleY), text, notif.title.c_str());

        ImGui::PopFont();

        // 시간 그리기
        drawList->AddText(ImVec2(drawMax.x - kPadding - timeSize.x, titleY), subText, notif.timestamp.c_str());




        // 내용 그리기
        while (text2 < textEnd)
        {
            const char* lineEnd = ImGui::GetFont()->CalcWordWrapPositionA(1.0f, text2, textEnd, wrapWidth);

            // **수정: 알파가 적용된 'text' 변수를 사용합니다.**
            drawList->AddText(ImVec2(contentStart.x + CONTENT_OFFSET.x, y), text, std::string(text2, lineEnd).c_str());

            y += lineHeight;
            text2 = lineEnd;
            while (*text2 == ' ' || *text2 == '\n') ++text2; // skip space or newline
        }

        posY -= (cardHeight + kSpacingY);
    }
}