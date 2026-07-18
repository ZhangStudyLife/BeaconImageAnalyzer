#include <windows.h>
#include <wchar.h>


int WINAPI wWinMain(HINSTANCE instance, HINSTANCE previous, PWSTR command_line, int show_command)
{
    wchar_t executable_path[MAX_PATH];
    wchar_t tools_dir[MAX_PATH];
    wchar_t root_dir[MAX_PATH];
    wchar_t script_path[MAX_PATH];
    wchar_t python_path[MAX_PATH];
    wchar_t child_command[MAX_PATH * 3];
    STARTUPINFOW startup_info;
    PROCESS_INFORMATION process_info;
    wchar_t *separator;

    (void)instance;
    (void)previous;
    (void)command_line;
    (void)show_command;

    if(GetModuleFileNameW(NULL, executable_path, MAX_PATH) == 0)
    {
        MessageBoxW(NULL, L"无法获取启动器路径。", L"信标算法参数对比", MB_OK | MB_ICONERROR);
        return 1;
    }

    wcscpy_s(tools_dir, MAX_PATH, executable_path);
    separator = wcsrchr(tools_dir, L'\\');
    if(separator == NULL)
    {
        return 1;
    }
    *separator = L'\0';

    wcscpy_s(root_dir, MAX_PATH, tools_dir);
    separator = wcsrchr(root_dir, L'\\');
    if(separator == NULL)
    {
        return 1;
    }
    *separator = L'\0';

    swprintf_s(script_path, MAX_PATH, L"%ls\\parameter_compare_gui.py", tools_dir);
    if(GetEnvironmentVariableW(L"PARAMETER_COMPARE_PYTHON", python_path, MAX_PATH) == 0)
    {
        wcscpy_s(python_path, MAX_PATH, L"pythonw.exe");
    }
    swprintf_s(child_command, MAX_PATH * 3, L"\"%ls\" \"%ls\"", python_path, script_path);

    ZeroMemory(&startup_info, sizeof(startup_info));
    startup_info.cb = sizeof(startup_info);
    ZeroMemory(&process_info, sizeof(process_info));
    if(!CreateProcessW(NULL,
                       child_command,
                       NULL,
                       NULL,
                       FALSE,
                       0,
                       NULL,
                       root_dir,
                       &startup_info,
                       &process_info))
    {
        MessageBoxW(NULL,
                    L"无法启动 Python 界面。请确认 pythonw.exe 在 PATH 中，或设置 PARAMETER_COMPARE_PYTHON。",
                    L"信标算法参数对比",
                    MB_OK | MB_ICONERROR);
        return 1;
    }

    CloseHandle(process_info.hThread);
    CloseHandle(process_info.hProcess);
    return 0;
}
