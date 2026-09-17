from __future__ import annotations

from fastapi import APIRouter, Depends, Request, Response

from pinky_control_center.auth import CSRF_COOKIE, SESSION_COOKIE, current_user, verify_mutation
from pinky_control_center.models import LoginRequest, LoginResponse, UserInfo

router = APIRouter(prefix="/api/v1")


@router.post("/session", response_model=LoginResponse)
async def login(payload: LoginRequest, request: Request, response: Response) -> LoginResponse:
    if request.headers.get("origin") != request.app.state.allowed_origin:
        from fastapi import HTTPException
        raise HTTPException(status_code=403, detail="ORIGIN_FORBIDDEN")
    user = request.app.state.storage.authenticate(payload.username, payload.password)
    if user is None and getattr(request.app.state, "auth_bypass", False):
        # Explicit operator-approved bypass (CONTROL_PLATFORM_AUTH_BYPASS=1):
        # issue a normal session for an existing account without verifying
        # the password. Unknown usernames are still rejected, and every
        # other gate (Origin, CSRF, lease, stop latch) is unchanged.
        user = request.app.state.storage.find_user(payload.username)
    if user is None:
        from fastapi import HTTPException
        raise HTTPException(status_code=401, detail="AUTH_REQUIRED")
    token, csrf = request.app.state.storage.create_session(user)
    secure = bool(getattr(request.app.state, "secure_cookies", False))
    response.set_cookie(SESSION_COOKIE, token, httponly=True, samesite="lax", secure=secure, max_age=8 * 3600)
    response.set_cookie(CSRF_COOKIE, csrf, httponly=False, samesite="lax", secure=secure, max_age=8 * 3600)
    return LoginResponse(user=user, csrf_token=csrf)


@router.get("/session", response_model=UserInfo)
async def session(user: UserInfo = Depends(current_user)) -> UserInfo:
    return user


@router.delete("/session", status_code=204)
async def logout(request: Request, response: Response, user: UserInfo = Depends(current_user)) -> Response:
    verify_mutation(request, user)
    request.app.state.storage.delete_session(request.cookies.get(SESSION_COOKIE))
    secure = bool(getattr(request.app.state, "secure_cookies", False))
    response.delete_cookie(SESSION_COOKIE, secure=secure, samesite="lax")
    response.delete_cookie(CSRF_COOKIE, secure=secure, samesite="lax")
    response.status_code = 204
    return response
